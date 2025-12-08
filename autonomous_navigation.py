import math
import sys
import os
import threading
import time
import asyncio
from queue import Queue
import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector
from bleak import BleakClient
from pid_controller import PID

# ==================== CONFIGURATION ====================
# Camera settings
CAMERA_ID = 2
IMAGE_RES = (1920, 1080)
CALIBRATION_FILE = 'camera_calibration.yaml'

# AprilTag settings
TAG_FAMILY = 'tag36h11'
TAG_SIZE = 0.0475  # 47.5mm
MIN_DECISION_MARGIN = 50.0
ROBOT_TAG_ID = 0
WINDOW     = "AprilTag GUI Tracker"

# Bluetooth settings
BT_ADDRESS = "B0:D2:78:32:EA:6C"  # HM-10 MAC
BT_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Navigation control parameters
HEADING_THRESHOLD = 5.0  # degrees - target reached if within this angle
TRANSLATIONAL_THRESHOLD = 0.015 # meters - waypoint reached if within this
TURN_SPEED = 0.35  # Motor speed for turning (0.0 to 1.0)
MIN_TURN_SPEED = 0.25  # Minimum speed to ensure movement
CONTROL_RATE = 50  # Hz - control loop update rate
Kp = 0.008  # Proportional gain for heading correction
Ki = 0.0
Kd = 0.0

# Navigation states
STATE_IDLE = 0
STATE_DRIVING = 1
STATE_ARRIVED = 2
BOX_SIZE  = 30
# =======================================================

def load_camera_calibration(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return np.array(data["camera_matrix"]), np.array(data["distortion_coefficients"])

def px_to_meters(x_px, y_px, Z, K):
    """
    Convert image pixel (x_px, y_px) at depth Z (m) into camera-frame meters.
    K: 3x3 camera matrix
    Returns (X_m, Y_m) in meters, using OpenCV camera coords (x right, y down, z forward).
    """
    fx = K[0, 0]; fy = K[1, 1]; cx = K[0, 2]; cy = K[1, 2]
    X = (x_px - cx) * (Z / fx)
    Y = (y_px - cy) * (Z / fy)
    return X, Y

def initialize_camera(camera_id, resolution):
    """Initialize camera."""
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)
    
    print(f"✓ Camera initialized: {resolution[0]}x{resolution[1]}")
    return cap


def rotation_matrix_to_euler_angles(R):
    """Convert rotation matrix to Euler angles (roll, pitch, yaw) in degrees."""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    
    return math.degrees(x), math.degrees(y), math.degrees(z)



class MotionCaptureThread(threading.Thread):
    def __init__(self, shared_state, nav_shared_state):
        """
        Initialize motion capture thread.
        
        Args:
            shared_state: Shared data structure for robot state
        """
        super().__init__(daemon=True)
        self.shared_state = shared_state
        self.nav_shared_state = nav_shared_state
        self.running = False
        
        # Camera and detector
        self.cap = None
        self.detector = None
        self.camera_matrix = None
        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, self.mouse_callback)
    
    def initialize(self):
        """Initialize camera and detector."""
        # Load calibration
        self.camera_matrix, dist_coeffs = load_camera_calibration(CALIBRATION_FILE)
        if self.camera_matrix is None:
            return False
        
        # Initialize camera
        self.cap = initialize_camera(CAMERA_ID, IMAGE_RES)
        if self.cap is None:
            return False
        
        # Initialize detector
        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.sp_x = IMAGE_RES[0] // 2
        self.sp_y = IMAGE_RES[1] // 2
        print("✓ Motion capture initialized")
        
        return True
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.sp_x, self.sp_y = x, y
            print("New waypoint:", x, y)
            with self.nav_shared_state['lock']:
                self.nav_shared_state['state'] = STATE_DRIVING
    
    def run(self):
        """Main tracking loop - runs continuously at high speed."""
        self.running = True
        
        while self.running:
            # Capture frame
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Convert to grayscale
            frame[frame < 80] = 0
            frame[frame > 200] = 255
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Detect tags
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[
                    self.camera_matrix[0, 0],
                    self.camera_matrix[1, 1],
                    self.camera_matrix[0, 2],
                    self.camera_matrix[1, 2],
                ],
                tag_size=TAG_SIZE
            )
            
            # Filter by quality
            #detections = [d for d in detections if d.decision_margin >= MIN_DECISION_MARGIN]
            cv2.rectangle(
                frame,
                (self.sp_x - BOX_SIZE, self.sp_y - BOX_SIZE),
                (self.sp_x + BOX_SIZE, self.sp_y + BOX_SIZE),
                (0, 255, 0), 2
            )
            if len(detections) > 0:
                robot_detection = detections[0]
                # Get pose
                x = robot_detection.pose_t[0, 0]
                y = robot_detection.pose_t[1, 0]
                z = robot_detection.pose_t[2, 0]
                
                roll, pitch, yaw = rotation_matrix_to_euler_angles(robot_detection.pose_R)
                sp_xm, sp_ym = px_to_meters(self.sp_x, self.sp_y, z, self.camera_matrix)
                # Update tracker
                rel_x = sp_xm - x
                rel_y = sp_ym - y
                target_yaw = math.degrees(math.atan2(rel_y, rel_x))
                rel_yaw = target_yaw - yaw # positive means turn left, negative means turn right
                rel_yaw = (rel_yaw + 180) % 360 - 180
                
                # Update shared state (thread-safe with lock)
                with self.shared_state['lock']:
                    self.shared_state['x'] = rel_x
                    self.shared_state['y'] = rel_y
                    self.shared_state['yaw'] = rel_yaw
                    self.shared_state['tracking'] = True
            else:
                # Lost tracking
                with self.shared_state['lock']:
                    self.shared_state['tracking'] = False
            cv2.imshow(WINDOW, frame)
            if cv2.waitKey(1) == 27:
                self.running = False
                break
    
    def stop(self):
        """Stop tracking."""
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyWindow(WINDOW)

class BluetoothControlThread(threading.Thread):
    def __init__(self, shared_state, command_queue):
        super().__init__(daemon=True)
        self.shared_state = shared_state
        self.command_queue = command_queue
        self.running = False
        self.client = None
        self.loop = None
    def run(self):
        """Main control loop - processes commands from queue."""
        self.running = True
        
        # Create new event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # Run async main
        self.loop.run_until_complete(self._async_main())
    
    async def _async_main(self):
        """Async main loop."""
        try:
            # Connect to robot
            print(f"Connecting to Bluetooth device {BT_ADDRESS}...")
            async with BleakClient(BT_ADDRESS) as client:
                self.client = client
                print(f"✓ Connected to robot via Bluetooth")
                
                with self.shared_state['lock']:
                    self.shared_state['bt_connected'] = True
                
                # Process commands from queue
                while self.running:
                    try:
                        # Check for new command (non-blocking)
                        if not self.command_queue.empty():
                            left_speed, right_speed = self.command_queue.get_nowait()
                            
                            # Send command to robot
                            command = f"{left_speed:.2f} {right_speed:.2f}\n"
                            await client.write_gatt_char(BT_CHAR_UUID, command.encode(), response=False)
                        
                        # Small delay to prevent CPU spinning
                        await asyncio.sleep(0.01)
                        
                    except Exception as e:
                        print(f"Command send error: {e}")
        
        except Exception as e:
            print(f"Bluetooth connection error: {e}")
            with self.shared_state['lock']:
                self.shared_state['bt_connected'] = False
    
    def stop(self):
        """Stop control loop."""
        self.running = False
        # Send stop command
        if self.client and self.client.is_connected:
            self.command_queue.put((0.0, 0.0))


class NavigationController:
    def __init__(self, 
                 shared_state, 
                 nav_shared_state, 
                 command_queue,
                 max_power=0.6,
                 v_cruise=0.6,
                 dist_kp=0.8, dist_ki=0.0, dist_kd=0.1,
                 head_kp=2.0, head_ki=0.0, head_kd=0.1
                 ):
        """
        Initialize navigation controller.
        
        Args:
            shared_state: Shared robot state
            command_queue: Queue for sending motor commands
        """
        self.shared_state = shared_state
        self.nav_shared_state = nav_shared_state
        self.command_queue = command_queue
        self.state = STATE_IDLE
        self.max_power = max_power   # max motor power (e.g. 0.6)
        self.v_cruise = v_cruise     # forward power for intermediate waypoints

        self.dist_pid = PID(dist_kp, dist_ki, dist_kd, integral_limit=2.0)
        self.head_pid = PID(head_kp, head_ki, head_kd, integral_limit=2.0)

    def reset(self):
        self.dist_pid.reset()
        self.head_pid.reset()
    
    def update(self, dt=0.02):
        """
        Update control loop (call at CONTROL_RATE Hz).
        """
        with self.nav_shared_state['lock']:
            self.state = self.nav_shared_state['state']
        if self.state == STATE_IDLE:
            return
        
        # Get current robot state (thread-safe)
        with self.shared_state['lock']:
            rel_x = self.shared_state['x']
            rel_y = self.shared_state['y']
            rel_yaw = self.shared_state['yaw']
            tracking = self.shared_state['tracking']
            bt_connected = self.shared_state['bt_connected']
        
        # Can't control if not tracking or not connected
        if not tracking or not bt_connected:
            self.stop_motors()
            return
        if self.state == STATE_ARRIVED and abs(rel_yaw) >= HEADING_THRESHOLD:
            self.state = STATE_DRIVING

        #TODO: FUTURE/NEXT STEPS. Implement multi waypoint navigation here



        #

        # navigation to final waypoint
        self.waypoint_navigate(rel_x, rel_y, rel_yaw, dt=dt, is_final=True)

    def waypoint_navigate(self, dx, dy, d_dir_deg, dt, is_final=False):
        """
        Docstring for waypoint_navigate
        
        :param dx: x displacement to target in meters
        :param dy: y displacement to target in meters
        :param d_dir_deg: yaw displacement to target in degrees
        :param is_final: whether or not this is the final waypoint.
                        If True, stops at waypoint. 
                        If False, follows through (keeping speed)
        """
        dist = math.hypot(dx, dy)
        d_dir = math.radians(d_dir_deg)
        if is_final and dist < TRANSLATIONAL_THRESHOLD:
            self.stop_motors()
            self.state = STATE_ARRIVED
            with self.nav_shared_state['lock']:
                self.nav_shared_state['state'] = STATE_ARRIVED
            return
        # tune heading pid
        #at_final = is_final and dist < TRANSLATIONAL_THRESHOLD  # meters threshold (tune)
        w_cmd = self.head_pid.step(d_dir, dt)
        w_cmd_max = 1.0
        w_cmd = max(-w_cmd_max, min(w_cmd, w_cmd_max))

        # tune linear pid
        if is_final:
            # Final waypoint: distance PID, slows down as dist → 0
            e_dist = dist
            v_cmd = self.dist_pid.step(e_dist, dt)

            # Never go backwards for final approach (optional):
            v_cmd = max(0.0, v_cmd)

            # Limit linear command
            v_cmd_max = 1.0
            v_cmd = max(-v_cmd_max, min(v_cmd, v_cmd_max))
        else:
            # Intermediate waypoint: constant cruising speed
            # Only reduce speed if you are pointing too far away from path
            # (optional condition - you can remove if you truly never want to slow)
            angle_mag = abs(d_dir)
            if angle_mag < math.radians(90):
                v_cmd = self.v_cruise / self.max_power  # normalized to ~1 range
            else:
                # if facing away badly, slow down
                v_cmd = 0.0
        
        #send motor commands
        left  = v_cmd - w_cmd
        right = v_cmd + w_cmd
        m = max(1.0, abs(left), abs(right))
        left  /= m
        right /= m
        left_power  = left * self.max_power
        right_power = right * self.max_power
        self.command_queue.put((left_power, right_power))
    
    def _control_turning(self, rel_x, rel_y, rel_yaw):
        """
        This is for turning towards a target
        """
        
        # Check if facing target
        if abs(rel_yaw) < HEADING_THRESHOLD:
            # Arrived at correct orientation
            self.stop_motors()
            self.state = STATE_ARRIVED
            with self.nav_shared_state['lock']:
                self.nav_shared_state['state'] = STATE_ARRIVED
            print(f"✓ Facing target! Error: {rel_yaw:.1f}°")
            with self.shared_state['lock']:
                self.shared_state['status'] = "Facing target"
            return
        
        # Calculate motor speeds with proportional control
        
        turn_correction = Kp * rel_yaw
        turn_correction = max(-TURN_SPEED, min(TURN_SPEED, turn_correction))  # Clamp
        
        # Differential drive: opposite wheel directions for in-place rotation
        if rel_yaw > 0:  # Turn left (counterclockwise)
            left_speed = -abs(turn_correction)
            right_speed = abs(turn_correction)
        else:  # Turn right (clockwise)
            left_speed = abs(turn_correction)
            right_speed = -abs(turn_correction)
        
        # Ensure minimum speed to overcome static friction
        if abs(left_speed) < MIN_TURN_SPEED and abs(left_speed) > 0:
            left_speed = MIN_TURN_SPEED * (1 if left_speed > 0 else -1)
        if abs(right_speed) < MIN_TURN_SPEED and abs(right_speed) > 0:
            right_speed = MIN_TURN_SPEED * (1 if right_speed > 0 else -1)
        
        # Send motor command
        self.command_queue.put((left_speed, right_speed))
        
        # Update status
        with self.shared_state['lock']:
            self.shared_state['status'] = f"Turning (error: {rel_yaw:+.1f}°)"
    
    def stop_motors(self):
        """Stop all motors."""
        self.command_queue.put((0.0, 0.0))

        

def main():
    """Main entry point."""
    shared_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'tracking': False,
            'bt_connected': False,
            'status': 'Initializing',
            'lock': threading.Lock()
        }
    
    nav_shared_state = {
        'state': STATE_IDLE,
        'lock': threading.Lock()
    }

    command_queue = Queue()
    navigation = NavigationController(shared_state, nav_shared_state, command_queue)
    motion_capture = MotionCaptureThread(shared_state, nav_shared_state)
    bluetooth_control = BluetoothControlThread(shared_state, command_queue)
    
    # Start threads
    if motion_capture.initialize():
        motion_capture.start()
        print("✓ Motion capture thread started")
    
    bluetooth_control.start()
    print("✓ Bluetooth control thread started")

    navigation.state = STATE_IDLE

    try:
        dt = 1.0 / CONTROL_RATE
        while True:
            navigation.update(dt=dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        motion_capture.stop()
        bluetooth_control.stop()


if __name__ == '__main__':
    main()