"""
autonomous_navigation.py

Autonomous robot navigation system with closed-loop control.

Features:
- Multi-threaded motion capture (fast position tracking)
- Bluetooth motor control
- Closed-loop orientation control
- Turn-to-target with feedback correction
- GUI for target selection and visualization

The system continuously:
1. Tracks robot position/orientation from camera
2. Calculates required heading to target
3. Adjusts motor speeds to correct orientation
4. Displays real-time feedback on GUI
"""

import tkinter as tk
from tkinter import ttk, messagebox
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

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from motion_capture.position_tracker import RelativePositionTracker


# ==================== CONFIGURATION ====================
# Camera settings
CAMERA_ID = 1
IMAGE_RES = (640, 480)
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), 'motion_capture', 'camera_calibration.yaml')

# AprilTag settings
TAG_FAMILY = 'tag36h11'
TAG_SIZE = 0.0475  # 47.5mm
MIN_DECISION_MARGIN = 50.0
ROBOT_TAG_ID = 0

# Bluetooth settings
BT_ADDRESS = "B0:D2:78:32:EA:6C"  # HM-10 MAC
BT_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# GUI settings
CANVAS_SIZE = 600  # pixels
WORLD_SIZE = 40.0  # cm (±20cm range)

# Navigation control parameters
HEADING_THRESHOLD = 5.0  # degrees - target reached if within this angle
TURN_SPEED = 0.35  # Motor speed for turning (0.0 to 1.0)
MIN_TURN_SPEED = 0.25  # Minimum speed to ensure movement
CONTROL_RATE = 30  # Hz - control loop update rate
Kp = 0.02  # Proportional gain for heading correction

# Navigation states
STATE_IDLE = 0
STATE_TURNING = 1
STATE_ARRIVED = 2
# =======================================================


def load_camera_calibration(calibration_file):
    """Load camera calibration from YAML file."""
    try:
        with open(calibration_file, 'r') as f:
            calib_data = yaml.safe_load(f)
        
        camera_matrix = np.array(calib_data['camera_matrix'])
        dist_coeffs = np.array(calib_data['distortion_coefficients'])
        
        print("✓ Camera calibration loaded")
        return camera_matrix, dist_coeffs
        
    except FileNotFoundError:
        print(f"Error: Calibration file '{calibration_file}' not found!")
        return None, None
    except Exception as e:
        print(f"Error loading calibration: {e}")
        return None, None


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


def normalize_angle(angle):
    """Normalize angle to [-180, 180] range."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


class MotionCaptureThread(threading.Thread):
    """
    High-speed motion capture thread.
    Continuously tracks robot position and updates shared state.
    """
    
    def __init__(self, shared_state):
        """
        Initialize motion capture thread.
        
        Args:
            shared_state: Shared data structure for robot state
        """
        super().__init__(daemon=True)
        self.shared_state = shared_state
        self.running = False
        
        # Camera and detector
        self.cap = None
        self.detector = None
        self.tracker = None
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
    def initialize(self):
        """Initialize camera and detector."""
        # Load calibration
        self.camera_matrix, dist_coeffs = load_camera_calibration(CALIBRATION_FILE)
        if self.camera_matrix is None:
            return False
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
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
        
        # Initialize tracker
        self.tracker = RelativePositionTracker(ROBOT_TAG_ID)
        
        print("✓ Motion capture initialized")
        return True
    
    def run(self):
        """Main tracking loop - runs continuously at high speed."""
        self.running = True
        
        while self.running:
            try:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    continue
                
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (3, 3), 0)
                
                # Detect tags
                detections = self.detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=[self.fx, self.fy, self.cx, self.cy],
                    tag_size=TAG_SIZE
                )
                
                # Filter by quality
                detections = [d for d in detections if d.decision_margin >= MIN_DECISION_MARGIN]
                
                # Find robot tag
                robot_detection = None
                for detection in detections:
                    if detection.tag_id == ROBOT_TAG_ID:
                        robot_detection = detection
                        break
                
                # Update tracking
                if robot_detection:
                    # Get pose
                    position = (
                        robot_detection.pose_t[0, 0],
                        robot_detection.pose_t[1, 0],
                        robot_detection.pose_t[2, 0]
                    )
                    
                    roll, pitch, yaw = rotation_matrix_to_euler_angles(robot_detection.pose_R)
                    orientation = (roll, pitch, yaw)
                    
                    # Update tracker
                    rel_x, rel_y, rel_z, rel_yaw = self.tracker.update(position, orientation)
                    
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
                
            except Exception as e:
                print(f"Motion capture error: {e}")
    
    def stop(self):
        """Stop tracking."""
        self.running = False
        if self.cap:
            self.cap.release()


class BluetoothControlThread(threading.Thread):
    """
    Bluetooth motor control thread.
    Handles asynchronous BLE communication with robot.
    """
    
    def __init__(self, shared_state, command_queue):
        """
        Initialize Bluetooth control thread.
        
        Args:
            shared_state: Shared data structure for robot state
            command_queue: Queue for motor commands
        """
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
    """
    Navigation control logic.
    Calculates motor commands based on robot position and target.
    """
    
    def __init__(self, shared_state, command_queue):
        """
        Initialize navigation controller.
        
        Args:
            shared_state: Shared robot state
            command_queue: Queue for sending motor commands
        """
        self.shared_state = shared_state
        self.command_queue = command_queue
        self.state = STATE_IDLE
        self.target_x = None
        self.target_y = None
        self.target_heading = None
        
    def set_target(self, x, y):
        """
        Set new target position.
        
        Args:
            x: Target X in cm
            y: Target Y in cm
        """
        self.target_x = x
        self.target_y = y
        self.state = STATE_TURNING
        print(f"New target set: ({x:.1f}, {y:.1f})")
    
    def update(self):
        """
        Update control loop (call at CONTROL_RATE Hz).
        """
        if self.state == STATE_IDLE:
            return
        
        # Get current robot state (thread-safe)
        with self.shared_state['lock']:
            robot_x = self.shared_state['x']
            robot_y = self.shared_state['y']
            robot_yaw = self.shared_state['yaw']
            tracking = self.shared_state['tracking']
            bt_connected = self.shared_state['bt_connected']
        
        # Can't control if not tracking or not connected
        if not tracking or not bt_connected:
            self.stop_motors()
            return
        
        if self.state == STATE_TURNING:
            self._control_turning(robot_x, robot_y, robot_yaw)
    
    def _control_turning(self, robot_x, robot_y, robot_yaw):
        """
        Control turning to face target.
        
        Args:
            robot_x: Current X position in cm
            robot_y: Current Y position in cm
            robot_yaw: Current yaw in degrees
        """
        # Calculate desired heading to target
        dx = self.target_x - robot_x
        dy = self.target_y - robot_y
        self.target_heading = math.degrees(math.atan2(dy, dx))
        
        # Calculate heading error
        heading_error = normalize_angle(self.target_heading - robot_yaw)
        
        # Check if facing target
        if abs(heading_error) < HEADING_THRESHOLD:
            # Arrived at correct orientation
            self.stop_motors()
            self.state = STATE_ARRIVED
            print(f"✓ Facing target! Error: {heading_error:.1f}°")
            with self.shared_state['lock']:
                self.shared_state['status'] = "Facing target"
            return
        
        # Calculate motor speeds with proportional control
        # Positive error = turn left (left motor slower/backward, right motor faster/forward)
        # Negative error = turn right (left motor faster/forward, right motor slower/backward)
        
        turn_correction = Kp * heading_error
        turn_correction = max(-TURN_SPEED, min(TURN_SPEED, turn_correction))  # Clamp
        
        # Differential drive: opposite wheel directions for in-place rotation
        if heading_error > 0:  # Turn left (counterclockwise)
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
            self.shared_state['status'] = f"Turning (error: {heading_error:+.1f}°)"
    
    def stop_motors(self):
        """Stop all motors."""
        self.command_queue.put((0.0, 0.0))
    
    def clear_target(self):
        """Clear current target."""
        self.stop_motors()
        self.target_x = None
        self.target_y = None
        self.state = STATE_IDLE
        with self.shared_state['lock']:
            self.shared_state['status'] = "Idle"


class AutonomousNavigationGUI:
    """
    GUI for autonomous navigation with closed-loop control.
    """
    
    def __init__(self, root):
        """
        Initialize GUI.
        
        Args:
            root: Tkinter root window
        """
        self.root = root
        
        # Shared state for multi-threading (thread-safe)
        self.shared_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'tracking': False,
            'bt_connected': False,
            'status': 'Initializing',
            'lock': threading.Lock()
        }
        
        # Command queue for motor commands
        self.command_queue = Queue()
        
        # Canvas settings
        self.canvas_size = CANVAS_SIZE
        self.world_size = WORLD_SIZE
        self.scale = self.canvas_size / self.world_size
        
        # Target
        self.target_x = None
        self.target_y = None
        
        # Setup GUI
        self.setup_ui()
        
        # Initialize threads
        self.motion_capture = MotionCaptureThread(self.shared_state)
        self.bluetooth_control = BluetoothControlThread(self.shared_state, self.command_queue)
        self.navigation = NavigationController(self.shared_state, self.command_queue)
        
        # Start threads
        if self.motion_capture.initialize():
            self.motion_capture.start()
            print("✓ Motion capture thread started")
        
        self.bluetooth_control.start()
        print("✓ Bluetooth control thread started")
        
        # Start update loops
        self.update_display()
        self.update_navigation()
        
    def setup_ui(self):
        """Create GUI elements."""
        self.root.title("Autonomous Robot Navigation")
        self.root.geometry("900x750")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, text="Autonomous Navigation with Closed-Loop Control", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Instructions
        instructions = ttk.Label(main_frame, 
                                text="Click to set target - Robot will automatically turn to face it",
                                font=('Arial', 10))
        instructions.grid(row=1, column=0, columnspan=2, pady=5)
        
        # Canvas
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=2, column=0, padx=10, pady=10)
        
        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_size, 
                               height=self.canvas_size, bg='white', 
                               relief=tk.SUNKEN, borderwidth=2)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Robot Status", padding="10")
        control_frame.grid(row=2, column=1, padx=10, pady=10, sticky=(tk.N, tk.W, tk.E))
        
        # Connection status
        ttk.Label(control_frame, text="System Status:", 
                 font=('Arial', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)
        
        self.tracking_label = ttk.Label(control_frame, text="⚫ Camera: Initializing", 
                                        font=('Arial', 9))
        self.tracking_label.grid(row=1, column=0, columnspan=2, pady=2, sticky=tk.W)
        
        self.bt_label = ttk.Label(control_frame, text="⚫ Bluetooth: Connecting", 
                                  font=('Arial', 9))
        self.bt_label.grid(row=2, column=0, columnspan=2, pady=2, sticky=tk.W)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Position
        ttk.Label(control_frame, text="Robot Position:", 
                 font=('Arial', 10, 'bold')).grid(row=4, column=0, columnspan=2, pady=5)
        
        ttk.Label(control_frame, text="X:").grid(row=5, column=0, sticky=tk.E)
        self.x_label = ttk.Label(control_frame, text="0.00 cm", font=('Arial', 10))
        self.x_label.grid(row=5, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Y:").grid(row=6, column=0, sticky=tk.E)
        self.y_label = ttk.Label(control_frame, text="0.00 cm", font=('Arial', 10))
        self.y_label.grid(row=6, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Yaw:").grid(row=7, column=0, sticky=tk.E)
        self.yaw_label = ttk.Label(control_frame, text="0.00°", font=('Arial', 10))
        self.yaw_label.grid(row=7, column=1, sticky=tk.W, padx=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=8, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Navigation status
        ttk.Label(control_frame, text="Navigation:", 
                 font=('Arial', 10, 'bold')).grid(row=9, column=0, columnspan=2, pady=5)
        
        self.nav_status_label = ttk.Label(control_frame, text="Idle", 
                                          font=('Arial', 10))
        self.nav_status_label.grid(row=10, column=0, columnspan=2, pady=5)
        
        self.target_label = ttk.Label(control_frame, text="No target", 
                                      font=('Arial', 9))
        self.target_label.grid(row=11, column=0, columnspan=2, pady=5)
        
        self.heading_error_label = ttk.Label(control_frame, text="--", 
                                            font=('Arial', 9))
        self.heading_error_label.grid(row=12, column=0, columnspan=2, pady=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=13, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Buttons
        ttk.Button(control_frame, text="Stop Motors", 
                  command=self.emergency_stop).grid(row=14, column=0, columnspan=2, 
                                                    pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(control_frame, text="Clear Target", 
                  command=self.clear_target).grid(row=15, column=0, columnspan=2, 
                                                 pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(control_frame, text="Reset Origin", 
                  command=self.reset_origin).grid(row=16, column=0, columnspan=2, 
                                                  pady=5, sticky=(tk.W, tk.E))
        
        # Status bar
        self.status_label = ttk.Label(main_frame, text="System initializing...", 
                                     relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.grid(row=3, column=0, columnspan=2, 
                              sticky=(tk.W, tk.E), pady=5)
        
        # Draw grid
        self.draw_grid()
    
    def draw_grid(self):
        """Draw coordinate grid."""
        self.canvas.delete("grid")
        
        for i in range(-20, 21, 2):
            x = self.world_to_canvas_x(i)
            self.canvas.create_line(x, 0, x, self.canvas_size, fill='lightgray', tags="grid")
            
            y = self.world_to_canvas_y(i)
            self.canvas.create_line(0, y, self.canvas_size, y, fill='lightgray', tags="grid")
        
        center_x = self.world_to_canvas_x(0)
        center_y = self.world_to_canvas_y(0)
        
        self.canvas.create_line(center_x, 0, center_x, self.canvas_size, 
                               fill='gray', width=2, tags="grid")
        self.canvas.create_line(0, center_y, self.canvas_size, center_y, 
                               fill='gray', width=2, tags="grid")
        
        for i in range(-20, 21, 5):
            if i != 0:
                x = self.world_to_canvas_x(i)
                y = self.world_to_canvas_y(i)
                self.canvas.create_text(x, self.canvas_size - 10, text=f"{i}", tags="grid")
                self.canvas.create_text(10, y, text=f"{i}", tags="grid")
        
        self.canvas.create_text(center_x + 15, center_y - 15, 
                               text="(0,0)", tags="grid", font=('Arial', 10, 'bold'))
    
    def world_to_canvas_x(self, world_x):
        return int((world_x + 20) * self.scale)
    
    def world_to_canvas_y(self, world_y):
        return int((20 - world_y) * self.scale)
    
    def canvas_to_world_x(self, canvas_x):
        return (canvas_x / self.scale) - 20
    
    def canvas_to_world_y(self, canvas_y):
        return 20 - (canvas_y / self.scale)
    
    def draw_robot(self, x, y, yaw):
        """Draw robot."""
        self.canvas.delete("robot")
        
        cx = self.world_to_canvas_x(x)
        cy = self.world_to_canvas_y(y)
        
        radius = 15
        self.canvas.create_oval(cx - radius, cy - radius, 
                               cx + radius, cy + radius,
                               fill='blue', outline='darkblue', width=2, tags="robot")
        
        yaw_rad = math.radians(yaw)
        arrow_length = radius + 10
        end_x = cx + arrow_length * math.cos(yaw_rad)
        end_y = cy - arrow_length * math.sin(yaw_rad)
        
        self.canvas.create_line(cx, cy, end_x, end_y, 
                               fill='yellow', width=3, arrow=tk.LAST, tags="robot")
        
        self.canvas.create_text(cx, cy + radius + 15, 
                               text=f"({x:.1f}, {y:.1f})",
                               tags="robot", font=('Arial', 9))
    
    def draw_target(self):
        """Draw target."""
        self.canvas.delete("target")
        
        if self.target_x is not None and self.target_y is not None:
            tx = self.world_to_canvas_x(self.target_x)
            ty = self.world_to_canvas_y(self.target_y)
            
            size = 10
            self.canvas.create_line(tx - size, ty, tx + size, ty, 
                                   fill='red', width=2, tags="target")
            self.canvas.create_line(tx, ty - size, tx, ty + size, 
                                   fill='red', width=2, tags="target")
            
            self.canvas.create_oval(tx - size, ty - size, 
                                   tx + size, ty + size,
                                   outline='red', width=2, tags="target")
            
            self.canvas.create_text(tx, ty - size - 15, 
                                   text=f"Target: ({self.target_x:.1f}, {self.target_y:.1f})",
                                   tags="target", fill='red', font=('Arial', 9, 'bold'))
            
            # Draw heading line from robot to target
            with self.shared_state['lock']:
                robot_x = self.shared_state['x']
                robot_y = self.shared_state['y']
            
            rx = self.world_to_canvas_x(robot_x)
            ry = self.world_to_canvas_y(robot_y)
            self.canvas.create_line(rx, ry, tx, ty, 
                                   fill='green', width=2, dash=(5, 3), tags="target")
    
    def on_canvas_click(self, event):
        """Handle canvas click."""
        world_x = round(self.canvas_to_world_x(event.x), 1)
        world_y = round(self.canvas_to_world_y(event.y), 1)
        
        if world_x < -20 or world_x > 20 or world_y < -20 or world_y > 20:
            messagebox.showwarning("Out of Bounds", "Target must be within ±20cm range")
            return
        
        self.target_x = world_x
        self.target_y = world_y
        
        # Set target in navigation controller
        self.navigation.set_target(world_x, world_y)
        
        self.status_label.config(text=f"Target set at ({world_x:.1f}, {world_y:.1f}) - Turning...")
    
    def update_display(self):
        """Update display (30 Hz)."""
        # Get current state
        with self.shared_state['lock']:
            x = self.shared_state['x']
            y = self.shared_state['y']
            yaw = self.shared_state['yaw']
            tracking = self.shared_state['tracking']
            bt_connected = self.shared_state['bt_connected']
            status = self.shared_state['status']
        
        # Update labels
        self.x_label.config(text=f"{x:+.2f} cm")
        self.y_label.config(text=f"{y:+.2f} cm")
        self.yaw_label.config(text=f"{yaw:+.1f}°")
        
        if tracking:
            self.tracking_label.config(text="● Camera: Tracking", foreground='green')
        else:
            self.tracking_label.config(text="⚫ Camera: No signal", foreground='orange')
        
        if bt_connected:
            self.bt_label.config(text="● Bluetooth: Connected", foreground='green')
        else:
            self.bt_label.config(text="⚫ Bluetooth: Disconnected", foreground='red')
        
        self.nav_status_label.config(text=status)
        
        if self.target_x is not None:
            self.target_label.config(text=f"Target: ({self.target_x:.1f}, {self.target_y:.1f})")
            
            # Calculate heading error
            if self.navigation.target_heading is not None:
                heading_error = normalize_angle(self.navigation.target_heading - yaw)
                self.heading_error_label.config(text=f"Heading error: {heading_error:+.1f}°")
        else:
            self.target_label.config(text="No target")
            self.heading_error_label.config(text="--")
        
        # Draw
        if tracking:
            self.draw_robot(x, y, yaw)
        self.draw_target()
        
        # Schedule next update
        self.root.after(33, self.update_display)
    
    def update_navigation(self):
        """Update navigation control loop (30 Hz)."""
        self.navigation.update()
        
        # Schedule next update
        self.root.after(int(1000 / CONTROL_RATE), self.update_navigation)
    
    def emergency_stop(self):
        """Emergency stop."""
        self.navigation.stop_motors()
        self.status_label.config(text="Motors stopped")
    
    def clear_target(self):
        """Clear target."""
        self.navigation.clear_target()
        self.target_x = None
        self.target_y = None
        self.canvas.delete("target")
        self.status_label.config(text="Target cleared")
    
    def reset_origin(self):
        """Reset origin."""
        if hasattr(self.motion_capture, 'tracker') and self.motion_capture.tracker:
            self.motion_capture.tracker.reset()
            self.status_label.config(text="Origin reset")


def main():
    """Main entry point."""
    print("\n" + "="*70)
    print("AUTONOMOUS ROBOT NAVIGATION SYSTEM")
    print("="*70)
    print("\nFeatures:")
    print("  • Real-time motion capture tracking")
    print("  • Bluetooth motor control")
    print("  • Closed-loop orientation control")
    print("  • Automatic turn-to-target")
    print("\nClick on grid to set target. Robot will turn to face it.\n")
    
    root = tk.Tk()
    app = AutonomousNavigationGUI(root)
    
    def on_closing():
        if messagebox.askokcancel("Quit", "Stop robot and quit?"):
            app.navigation.stop_motors()
            app.motion_capture.stop()
            app.bluetooth_control.stop()
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    print("System ready. Click on grid to set targets.\n")
    root.mainloop()
    
    print("\nGoodbye!")


if __name__ == '__main__':
    main()
