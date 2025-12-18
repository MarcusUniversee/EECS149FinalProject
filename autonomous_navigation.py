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
import argparse
import csv

parser = argparse.ArgumentParser(description="Waypoint navigation")
parser.add_argument("-f", default=None, help="Filepath to waypoint yaml")
parser.add_argument("-l", default=None, help="Filepath to log file")
parser.add_argument("-o", "--out", default="recorded_waypoints.yaml", help="Output YAML path for recorded clicks")
# ==================== CONFIGURATION ====================
# Camera settings
CAMERA_ID = 0
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
TRANSLATIONAL_THRESHOLD = 0.05 # meters - waypoint reached if within this
TURN_SPEED = 0.35  # Motor speed for turning (0.0 to 1.0)
MIN_TURN_SPEED = 0.25  # Minimum speed to ensure movement
CONTROL_RATE = 50  # Hz - control loop update rate

MAX_VEL = 0.4 # need to change
MAX_ROT = 0.5
CRUISING_SPEED = 0.4 # need to change

DIST_KP = 0.9
DIST_KI = 0.0 
DIST_KD = 0.01
HEAD_KP = 0.045
HEAD_KI = 0.0
HEAD_KD = 0.0085

#Display parameter for smaller screens
DISPLAY_SCALE = 0.6

# Navigation states
STATE_IDLE = 0
STATE_DRIVING = 1
STATE_ARRIVED = 2
STATE_FINISHED = 3
BOX_SIZE  = 30

MIN_LIN_SPEED = 0.2 # need to change
MIN_ROT_SPEED = 0.02
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

def meters_to_px(X, Y, Z, K):
    """
    Convert camera-frame point (X, Y, Z) in meters into image pixel coordinates (x_px, y_px).
    K: 3x3 camera matrix
    Assumes OpenCV camera coords: x right, y down, z forward.
    Returns (x_px, y_px) as floats; cast to int if needed for indexing/drawing.
    """
    fx = K[0, 0]; fy = K[1, 1]; cx = K[0, 2]; cy = K[1, 2]

    if Z == 0:
        raise ValueError("Z must be nonzero to project to pixel coordinates")

    x_px = fx * (X / Z) + cx
    y_px = fy * (Y / Z) + cy
    return x_px, y_px

def show_frame(frame):
    vis = cv2.resize(frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_AREA)
    cv2.imshow(WINDOW, vis)

def initialize_camera(camera_id, resolution):
    """Initialize camera."""
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)#, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) #0.25 fo DSHOW
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)

    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])
    print(w, h, fps, fourcc_str)
    
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
    def __init__(self, shared_state, nav_shared_state, filename=None):
        """
        Initialize motion capture thread.
        
        Args:
            shared_state: Shared data structure for robot state
        """
        super().__init__(daemon=True)
        self.shared_state = shared_state
        self.nav_shared_state = nav_shared_state
        self.running = False
        
        self.waypoints_px = []   
        self.waypoints_m  = []  
        self.latest_z = None
        self.latest_yaw = None

        # Camera and detector
        self.cap = None
        self.detector = None
        self.camera_matrix = None
        self.filename = filename
        self.log = []
        self.start_log = False
        self.current = None
        cv2.namedWindow(WINDOW)
        if self.filename is None:
            cv2.setMouseCallback(WINDOW, self.mouse_callback)
    
    def initialize(self):
        """Initialize camera and detector."""
        # Load calibration
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(CALIBRATION_FILE)
        if self.camera_matrix is None:
            return False
        w, h = IMAGE_RES
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h),1.0, (w,h))
        self.new_camera_matrix = new_K
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2
        )
        
        # Initialize camera
        self.cap = initialize_camera(CAMERA_ID, IMAGE_RES)
        if self.cap is None:
            return False
        
        # Initialize detector
        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.sp_x = IMAGE_RES[0] // 2
        self.sp_y = IMAGE_RES[1] // 2
        self.sp_xm = 0
        self.sp_ym = 0
        self.current = None
        self.heading = None
        self.start_time = time.perf_counter()
        print("✓ Motion capture initialized")
        
        return True
    
    # def mouse_callback(self, event, x, y, flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         full_x = int(round(x / DISPLAY_SCALE))
    #         full_y = int(round(y / DISPLAY_SCALE))

    #         full_x = max(0, min(full_x, IMAGE_RES[0] - 1))
    #         full_y = max(0, min(full_y, IMAGE_RES[1] - 1))
    #         self.sp_x, self.sp_y = full_x, full_y
    #         print(f"New waypoint set: ({full_x}, {full_y}) from ({x}, {y}) - Press Enter to navigate")
    #         # Don't change state here - let main thread control it
    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        full_x = int(round(x / DISPLAY_SCALE))
        full_y = int(round(y / DISPLAY_SCALE))

        full_x = max(0, min(full_x, IMAGE_RES[0] - 1))
        full_y = max(0, min(full_y, IMAGE_RES[1] - 1))

        # Always move the "selection box" to the latest click
        self.sp_x, self.sp_y = full_x, full_y

        # Need a valid z from AprilTag pose to convert px -> meters
        if self.latest_z is None:
            return

        xm, ym = px_to_meters(full_x, full_y, self.latest_z, self.new_camera_matrix)

        # Record for drawing + export
        self.waypoints_px.append((full_x, full_y))

        # Store meters with rounding (nice YAML)
        wp = {
            "x": float(round(xm, 4)),
            "y": float(round(ym, 4)),
        }
        # Store start heading only (your file-mode expects heading on first waypoint)
        if len(self.waypoints_m) == 0:
            wp["heading"] = float(round(self.latest_yaw if self.latest_yaw is not None else 0.0, 1))
            wp["stop"] = False
        else:
            wp["stop"] = True

        self.waypoints_m.append(wp)

        print(f"Recorded waypoint #{len(self.waypoints_m)-1}: px=({full_x},{full_y})  m=({wp['x']},{wp['y']})")

    def export_waypoints_yaml(self, path):
        if not self.waypoints_m:
            print("No recorded waypoints to export.")
            return

        out_list = []

        # Name first as start, last as end, middle as wp1/wp2...
        n = len(self.waypoints_m)
        for i, wp in enumerate(self.waypoints_m):
            wp2 = dict(wp)  # copy
            if i == 0:
                wp2["name"] = "start"
                wp2["stop"] = False
                # heading already present from click
            elif i == n - 1:
                wp2["name"] = "end"
                wp2["stop"] = True
                wp2.pop("heading", None)  # keep heading only for start
            else:
                wp2["name"] = f"wp{i}"
                wp2["stop"] = True
                wp2.pop("heading", None)  # keep heading only for start

            # Ensure key order looks nice
            ordered = {}
            ordered["name"] = wp2["name"]
            ordered["x"] = wp2["x"]
            ordered["y"] = wp2["y"]
            if "heading" in wp2:
                ordered["heading"] = wp2["heading"]
            ordered["stop"] = wp2["stop"]

            out_list.append(ordered)

        data = {"waypoints": out_list}
        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)


    def run(self):
        """Main tracking loop - runs continuously at high speed."""
        self.running = True
        try:
            while self.running:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    continue
                # Convert to grayscale
                frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
                frame[frame < 50] = 0
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #gray = cv2.GaussianBlur(gray, (3, 3), 0)
                
                
                # Detect tags
                detections = self.detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=[
                        self.new_camera_matrix[0, 0],
                        self.new_camera_matrix[1, 1],
                        self.new_camera_matrix[0, 2],
                        self.new_camera_matrix[1, 2],
                    ],
                    tag_size=TAG_SIZE
                )
                cur_time = time.perf_counter() - self.start_time
                
                # Filter by quality
                #detections = [d for d in detections if d.decision_margin >= MIN_DECISION_MARGIN]
                cv2.rectangle(
                    frame,
                    (self.sp_x - BOX_SIZE, self.sp_y - BOX_SIZE),
                    (self.sp_x + BOX_SIZE, self.sp_y + BOX_SIZE),
                    (0, 255, 0), 2
                )

                if len(self.waypoints_px) >= 1:
                    for i, (px, py) in enumerate(self.waypoints_px):
                        cv2.circle(frame, (px, py), 5, (0, 255, 255), -1)
                        if i > 0:
                            pprev = self.waypoints_px[i - 1]
                            cv2.line(frame, pprev, (px, py), (0, 255, 255), 2)
                        cv2.putText(
                            frame, str(i), (px + 6, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
                            )
                if self.heading is not None:
                    L = 80
                    ex = int(self.sp_x + L * math.cos(math.radians(self.heading)))
                    ey = int(self.sp_y + L * math.sin(math.radians(self.heading)))
                    cv2.arrowedLine(
                        frame,
                        (self.sp_x, self.sp_y),
                        (ex, ey),
                        (255, 0, 255),   # red arrow
                        3,
                        tipLength=0.3
                    )
                if len(detections) > 0:
                    robot_detection = detections[0]
                    # Get pose
                    x = robot_detection.pose_t[0, 0]
                    y = robot_detection.pose_t[1, 0]
                    z = robot_detection.pose_t[2, 0]
                    
                    roll, pitch, yaw = rotation_matrix_to_euler_angles(robot_detection.pose_R)
                    
                    self.latest_z = float(z)
                    self.latest_yaw = float(yaw)

                    if self.filename is None:
                        self.sp_xm, self.sp_ym = px_to_meters(self.sp_x, self.sp_y, z, self.new_camera_matrix)
                    else:
                        x_px, y_px = meters_to_px(self.sp_xm, self.sp_ym, z, self.camera_matrix)
                        self.sp_x = int(round(x_px))
                        self.sp_y = int(round(y_px))

                    current = {
                        "t": cur_time,
                        "x": x,
                        "y": y,
                        "z": z,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "desired_x": self.sp_xm,
                        "desired_y": self.sp_ym
                    }
                    if self.start_log:
                        self.log.append(current)
                    self.current = current

                    try:
                        cv2.rectangle(
                            frame,
                            (self.sp_x - BOX_SIZE, self.sp_y - BOX_SIZE),
                            (self.sp_x + BOX_SIZE, self.sp_y + BOX_SIZE),
                            (0, 255, 0), 2
                        )
                    except Exception as e:
                        print(repr(e))
                    
                    #draw on gui
                    cv2.putText(
                        frame,
                        f"x: {x:.4f}, y: {y:.4f}, z: {z:.4f}, yaw: {yaw:.1f} deg",
                        (20, 40),                      # position (x, y)
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,                           # font scale
                        (0, 255, 0),                   # color (B, G, R)
                        2,                             # thickness
                        cv2.LINE_AA
                    )
                    corners = robot_detection.corners
                    cx = int(np.mean(corners[:, 0]))
                    cy = int(np.mean(corners[:, 1]))
                    angle_rad = math.radians(yaw)
                    L = 80
                    ex = int(cx + L * math.cos(angle_rad))
                    ey = int(cy + L * math.sin(angle_rad))
                    cv2.arrowedLine(
                        frame,
                        (cx, cy),
                        (ex, ey),
                        (0, 0, 255),   # red arrow
                        3,
                        tipLength=0.3
                    )
                    corners = corners.astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]), (255, 0, 0), 2)
                    # Update tracker
                    rel_x = self.sp_xm - x
                    rel_y = self.sp_ym - y
                    #print(self.heading)
                    if self.heading is None:
                        target_yaw = math.degrees(math.atan2(rel_y, rel_x))
                    else: 
                        target_yaw = self.heading
                    rel_yaw = yaw - target_yaw # positive means turn left, negative means turn right
                    rel_yaw = (rel_yaw + 180) % 360 - 180
                    if self.heading is not None:
                        cv2.putText(
                            frame,
                            f"Desired:  x: {self.sp_xm:.4f}, y: {self.sp_ym:.4f}, yaw: {self.heading:.1f} deg",
                            (20, 80),                      # position (x, y)
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,                           # font scale
                            (0, 0, 255),                   # color (B, G, R)
                            2,                             # thickness
                            cv2.LINE_AA
                        )
                    else:

                        cv2.putText(
                            frame,
                            f"Desired:  x: {self.sp_xm:.4f}, y: {self.sp_ym:.4f}, yaw: {target_yaw:.1f} deg",
                            (20, 80),                      # position (x, y)
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,                           # font scale
                            (0, 0, 255),                   # color (B, G, R)
                            2,                             # thickness
                            cv2.LINE_AA
                        )
                    
                    
                    # Update shared state (thread-safe with lock)
                    with self.shared_state['lock']:
                        self.shared_state['x'] = rel_x
                        self.shared_state['y'] = rel_y
                        self.shared_state['yaw'] = rel_yaw
                        self.shared_state['t'] = cur_time
                        self.shared_state['tracking'] = True
                        self.shared_state['frame'] = frame  # Store frame for display
                else:
                    # Lost tracking
                    with self.shared_state['lock']:
                        self.shared_state['tracking'] = False
                        self.shared_state['frame'] = frame # Store frame even when not tracking
        except Exception as e:
            print(repr(e))
    
    def set_waypoint(self, x, y, heading=None):
        self.sp_xm, self.sp_ym= float(x), float(y)
        if heading is not None:
            self.heading = float(heading)
        else:
            self.heading = None
        print("Waypoint (from file):", self.sp_xm, self.sp_ym)
        with self.nav_shared_state['lock']:
            self.nav_shared_state['state'] = STATE_DRIVING
    
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
                latest_cmd = None
                while self.running:
                    try:
                        while not self.command_queue.empty():
                            latest_cmd = self.command_queue.get()
                        
                        if latest_cmd is not None:
                            left_speed, right_speed = latest_cmd                            
                            
                            command = f"{left_speed:.2f} {right_speed:.2f}\n"
                            await client.write_gatt_char(BT_CHAR_UUID, command.encode(), response=False)
                    
                        # Small delay to prevent CPU spinning
                        await asyncio.sleep(0.02)
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
                 max_vel=0.6,
                 max_rot=0.6,
                 v_cruise=0.6,
                 dist_kp=2.8, dist_ki=0.0, dist_kd=0.1,
                 head_kp=2.0, head_ki=0.0, head_kd=0.5
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
        self.max_vel = max_vel   # max motor power (e.g. 0.6)
        self.max_rot = max_rot
        self.last_t = None 
        self.v_cruise = v_cruise     # forward power for intermediate waypoints

        self.dist_pid = PID(dist_kp, dist_ki, dist_kd, integral_limit=2.0)
        self.head_pid = PID(head_kp, head_ki, head_kd, integral_limit=2.0)
        self.is_final = False

    def reset(self):
        self.dist_pid.reset()
        self.head_pid.reset()
        self.last_t = None

    def set_is_final(self, is_final: bool):
        self.is_final = is_final
    
    def update(self, dt=0.02, is_stop=True):
        with self.nav_shared_state['lock']:
            self.state = self.nav_shared_state['state']
        if self.state == STATE_IDLE:
            return

        # Get current robot state (thread-safe)
        with self.shared_state['lock']:
            rel_x = self.shared_state['x']
            rel_y = self.shared_state['y']
            rel_yaw = self.shared_state['yaw']
            t = self.shared_state.get('t', None)
            tracking = self.shared_state['tracking']
            bt_connected = self.shared_state['bt_connected']

        if t is None:
            return

        # first valid timestamp: just initialize and wait for next frame
        if self.last_t is None:
            self.last_t = t
            return

        dt_meas = t - self.last_t
        if dt_meas <= 0:
            return
        self.last_t = t

        # clamp dt so one bad frame doesn't explode PID
        dt_meas = max(0.005, min(dt_meas, 0.1))

        if not tracking or not bt_connected:
            self.stop_motors()
            return

        self.waypoint_navigate(rel_x, rel_y, rel_yaw, dt=dt_meas, is_stop=is_stop)

    def waypoint_navigate(self, dx, dy, d_dir_deg, dt, is_stop=True):
        """
        Docstring for waypoint_navigate
        
        :param dx: x displacement to target in meters
        :param dy: y displacement to target in meters
        :param d_dir_deg: yaw displacement to target in degrees
        :param is_stop: whether or not this is a stopping waypoint.
                        If True, stops at waypoint. 
                        If False, follows through (keeping speed)
        """
        dist = math.hypot(dx, dy)
        d_dir = math.radians(d_dir_deg)
        if self.is_final and dist < TRANSLATIONAL_THRESHOLD:
            self.stop_motors()
            self.state = STATE_FINISHED
            with self.nav_shared_state['lock']:
                self.nav_shared_state['state'] = STATE_FINISHED
            return
        if dist < TRANSLATIONAL_THRESHOLD:
            if is_stop:
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

        if w_cmd > 0:
            w_cmd = min(w_cmd, self.max_rot)
        else:
            w_cmd = max(w_cmd, -self.max_rot)

        # tune linear pid
        # Final waypoint: distance PID, slows down as dist → 0
        e_dist = dist
        
        v_cmd = self.dist_pid.step(e_dist, dt)
        # Never go backwards for final approach (optional):
        v_cmd = max(0.0, v_cmd)

        # Limit linear command
        v_cmd_max = 1.0
        v_cmd = max(-v_cmd_max, min(v_cmd, v_cmd_max))
        angle_mag = abs(d_dir)
        if angle_mag < math.radians(15):
            v_cmd = min(self.v_cruise, v_cmd)
        v_cmd = min(v_cmd, self.max_vel)

        #multiwaypoint
        if not is_stop and abs(d_dir_deg) < HEADING_THRESHOLD*2:
            v_cmd = max(v_cmd, self.v_cruise)
        
        #send motor commands
        if abs(v_cmd) < MIN_LIN_SPEED and abs(v_cmd) > 0:
            v_cmd = MIN_LIN_SPEED * (1 if v_cmd > 0 else -1)
        if abs(w_cmd) < MIN_ROT_SPEED and abs(w_cmd) > 0:
            w_cmd = MIN_ROT_SPEED * (1 if w_cmd > 0 else -1)
        left  = v_cmd - w_cmd
        right = v_cmd + w_cmd
        m = max(1.0, abs(left), abs(right))
        left  /= m
        right /= m
        print(f"left: {left}, right: {right} ")
        left_power  = left
        right_power = right
        
        self.command_queue.put((left_power, right_power))
    
    def stop_motors(self):
        """Stop all motors."""
        self.command_queue.put((0.0, 0.0))

        

def main(filename=None, logfile=None, waypoints_out="recorded_waypoints.yaml"):
    """Main entry point."""
    shared_state = {
            'x': 10.0,
            'y': 10.0,
            'yaw': 0.0,
            't': -1.0,
            'tracking': False,
            'bt_connected': False,
            'status': 'Initializing',
            'frame': None,
            'lock': threading.Lock()
        }
    
    nav_shared_state = {
        'state': STATE_IDLE,
        'name': "",
        'lock': threading.Lock()
    }

    command_queue = Queue()
    navigation = NavigationController(shared_state, nav_shared_state, command_queue,
                 max_vel=MAX_VEL,
                 max_rot=MAX_ROT,
                 v_cruise=CRUISING_SPEED,
                 dist_kp=DIST_KP, dist_ki=DIST_KI, dist_kd=DIST_KD,
                 head_kp=HEAD_KP, head_ki=HEAD_KI, head_kd=HEAD_KD)
    motion_capture = MotionCaptureThread(shared_state, nav_shared_state, filename=filename)
    bluetooth_control = BluetoothControlThread(shared_state, command_queue)
    
    # Start threads
    if motion_capture.initialize():
        motion_capture.start()
        print("✓ Motion capture thread started")
    
    bluetooth_control.start()
    print("✓ Bluetooth control thread started")

    navigation.state = STATE_IDLE
    dt = 1.0 / CONTROL_RATE
    if filename is None:
        try:
            print("Click on the image to set a waypoint, then press Enter to navigate")
            print("Press ESC to exit")
            while True:
                # Display frame from motion capture thread
                with shared_state['lock']:
                    frame = shared_state.get('frame')
                
                if frame is not None:
                    show_frame(frame)
                
                # Wait for key press
                key = cv2.waitKey(1)
                if key == 13:  # Enter key
                    motion_capture.start_log = True
                    print("Starting navigation to waypoint...")
                    navigation.reset()  # Reset PID controllers
                    with nav_shared_state['lock']:
                        nav_shared_state['state'] = STATE_DRIVING
                    
                    # Navigate until arrived or finished
                    while True:
                        # Display frame
                        with shared_state['lock']:
                            frame = shared_state.get('frame')
                        if frame is not None:
                            show_frame(frame)
                        
                        with nav_shared_state['lock']:
                            state = nav_shared_state['state']
                        
                        if state == STATE_ARRIVED or state == STATE_FINISHED:
                            print("✓ Waypoint reached! Click to set another waypoint and press Enter")
                            navigation.stop_motors()
                            with nav_shared_state['lock']:
                                nav_shared_state['state'] = STATE_IDLE
                            break
                        
                        navigation.update(dt=dt)
                        time.sleep(dt)
                        
                        # Check for ESC key
                        if cv2.waitKey(1) == 27:  # ESC key
                            motion_capture.start_log = False
                            navigation.stop_motors()
                            motion_capture.stop()
                            bluetooth_control.stop()
                            raise KeyboardInterrupt
                
                elif key == 27:  # ESC key
                    motion_capture.start_log = False
                    navigation.stop_motors()
                    motion_capture.stop()
                    bluetooth_control.stop()
                    raise KeyboardInterrupt
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            print("logging")
            navigation.stop_motors()
            motion_capture.stop()
            bluetooth_control.stop()
            motion_capture.join()
            bluetooth_control.join()
            if logfile is not None and motion_capture.log:
                logs = motion_capture.log
                fieldnames = list(logs[0].keys())
                with open(logfile, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(logs)
            if waypoints_out is not None:
                motion_capture.export_waypoints_yaml(waypoints_out)
            
                        
    else:
        with open(filename, "r") as f:
            cfg = yaml.safe_load(f)
        waypoints = cfg.get("waypoints", [])
        if not waypoints:
            print("No waypoints found in waypoints.yaml")
            return
        else:
            print(f"Loaded {len(waypoints)} waypoints from file.")
        wp_idx = 0
        wp = waypoints[wp_idx]
        motion_capture.set_waypoint(wp["x"], wp["y"], heading=wp["heading"])
        print(wp["name"])
        print('Waiting for robot to be placed at starting waypoint')
        try:
            count = CONTROL_RATE * 2 #must be at starting waypoint for at least 2 seconds
            while True:
                with shared_state['lock']:
                    rel_x = shared_state['x']
                    rel_y = shared_state['y']
                    frame = shared_state.get('frame')
                    rel_yaw = shared_state['yaw']
                if frame is not None:
                    show_frame(frame)
                if math.hypot(rel_x, rel_y) < TRANSLATIONAL_THRESHOLD and abs(rel_yaw) < HEADING_THRESHOLD:
                    count -= 1
                    if count == 0:
                        print('ROBOT IN CORRECT POSITION')
                        break
                else:
                    count = CONTROL_RATE * 2
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC key
                    navigation.stop_motors()
                    motion_capture.stop()
                    bluetooth_control.stop()
                    raise KeyboardInterrupt
                if key == ord('q'):  # q key
                    print('SKIP ROBOT STARTING POINT')
                    break
                time.sleep(dt)
        except KeyboardInterrupt:
            motion_capture.stop()
            bluetooth_control.stop()
            return
            
            
        wp_idx = 1
        wp = waypoints[wp_idx]
        motion_capture.set_waypoint(wp["x"], wp["y"])
        navigation.set_is_final(wp_idx + 1 == len(waypoints))
        motion_capture.start_log = True
        time.sleep(dt)
        try:
            while True:
                with shared_state['lock']:
                    frame = shared_state.get('frame')
                if frame is not None:
                    show_frame(frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    raise KeyboardInterrupt
                # check to advance to next waypoint or not
                with nav_shared_state['lock']:
                    state = nav_shared_state['state']
                # print(state)
                if state == STATE_ARRIVED:
                    if wp_idx + 1 < len(waypoints):
                        wp_idx += 1
                        wp = waypoints[wp_idx]
                        print(f"Advancing to waypoint {wp_idx}: {wp}")

                        motion_capture.set_waypoint(wp["x"], wp["y"])
                        navigation.set_is_final(wp_idx + 1 == len(waypoints))
                        navigation.reset()  # Reset PID controllers for new waypoint

                        # resume driving
                        with nav_shared_state['lock']:
                            nav_shared_state['state'] = STATE_DRIVING
                    else:
                        # no more waypoints; go idle
                        with nav_shared_state['lock']:
                            nav_shared_state['state'] = STATE_IDLE
                        navigation.stop_motors()
                        print("All waypoints completed.")
                        break
                elif state == STATE_FINISHED:
                    motion_capture.start_log = False
                    navigation.stop_motors()
                    print("All waypoints completed.")
                    break
                navigation.update(dt=dt, is_stop=wp.get("stop", True))
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            print("logging")
            navigation.stop_motors()
            motion_capture.stop()
            bluetooth_control.stop()
            motion_capture.join()
            bluetooth_control.join()
            if logfile is not None and motion_capture.log:
                logs = motion_capture.log
                fieldnames = list(logs[0].keys())
                with open(logfile, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(logs)

            


if __name__ == '__main__':
    args = parser.parse_args()
    main(filename=args.f, logfile=args.l, waypoints_out=args.out)
