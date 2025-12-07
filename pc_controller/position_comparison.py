"""
position_comparison.py

Dual-window system comparing robot odometry (expected) vs motion capture (actual) positions.
Shows two 20cm x 20cm grids side-by-side:
- LEFT: Control window with clickable grid for navigation
- RIGHT: Trajectory comparison showing:
    * RED line: Expected trajectory (from odometry)
    * GREEN dots: Actual position (from motion capture, sampled every 1 second)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import sys
import time
import threading
import queue
from collections import deque
from datetime import datetime
import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector

# Add parent directory to path for imports
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pc_controller'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motion_capture'))

from robot_controller import RobotController


class MotionCaptureTracker:
    """
    Simplified AprilTag tracker for position comparison.
    Runs in background thread and provides position updates.
    """
    
    def __init__(self, camera_id=1, calibration_file='motion_capture/camera_calibration.yaml'):
        """
        Initialize motion capture tracker.
        
        Args:
            camera_id: Camera device ID
            calibration_file: Path to camera calibration YAML
        """
        self.camera_id = camera_id
        self.calibration_file = calibration_file
        self.running = False
        self.position_queue = queue.Queue(maxsize=10)
        
        # Current position (x, y in cm, z in meters)
        self.current_x_cm = 0.0
        self.current_y_cm = 0.0
        self.current_z_m = 0.0
        self.last_update_time = 0.0
        
        # AprilTag configuration
        self.TAG_FAMILY = 'tag16h5'
        self.TAG_SIZE = 0.023  # 23mm
        self.MIN_DECISION_MARGIN = 50.0
        
        # Camera and detector
        self.cap = None
        self.detector = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
    def load_camera_calibration(self):
        """Load camera calibration from YAML file."""
        try:
            with open(self.calibration_file, 'r') as f:
                data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['distortion_coefficients'])
                print(f"Loaded camera calibration from {self.calibration_file}")
                return True
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False
    
    def initialize_camera(self):
        """Initialize camera with optimized settings."""
        print(f"Opening camera {self.camera_id}...")
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.camera_id}")
            return False
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        print("Camera initialized successfully")
        return True
    
    def initialize_detector(self):
        """Initialize AprilTag detector."""
        self.detector = Detector(
            families=self.TAG_FAMILY,
            nthreads=2,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        print("AprilTag detector initialized")
        return True
    
    def start(self):
        """Start motion capture tracking in background thread."""
        if not self.load_camera_calibration():
            return False
        
        if not self.initialize_camera():
            return False
        
        if not self.initialize_detector():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.thread.start()
        print("Motion capture tracking started")
        return True
    
    def stop(self):
        """Stop motion capture tracking."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        print("Motion capture tracking stopped")
    
    def _tracking_loop(self):
        """Main tracking loop (runs in background thread)."""
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Detect AprilTags
            raw_detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[fx, fy, cx, cy],
                tag_size=self.TAG_SIZE
            )
            
            # Filter by quality
            detections = [d for d in raw_detections if d.decision_margin >= self.MIN_DECISION_MARGIN]
            
            if detections:
                # Use first detection
                detection = detections[0]
                
                # Get position (convert from meters to cm)
                x_m = detection.pose_t[0, 0]
                y_m = detection.pose_t[1, 0]
                z_m = detection.pose_t[2, 0]
                
                # Convert to cm
                x_cm = x_m * 100.0
                y_cm = y_m * 100.0
                
                # Update current position
                self.current_x_cm = x_cm
                self.current_y_cm = y_cm
                self.current_z_m = z_m
                self.last_update_time = time.time()
                
                # Put in queue (non-blocking)
                try:
                    self.position_queue.put_nowait((x_cm, y_cm, time.time()))
                except queue.Full:
                    pass  # Skip if queue full
            
            time.sleep(0.033)  # ~30 FPS
    
    def get_current_position(self):
        """
        Get most recent position from motion capture.
        
        Returns:
            tuple: (x_cm, y_cm, timestamp) or None if no recent data
        """
        if time.time() - self.last_update_time < 2.0:  # Data fresh within 2 seconds
            return (self.current_x_cm, self.current_y_cm, self.last_update_time)
        return None


class PositionComparisonGUI:
    """
    Dual-window GUI showing control interface and trajectory comparison.
    """
    
    def __init__(self, root, robot, mocap_tracker):
        """
        Initialize comparison GUI.
        
        Args:
            root: Tkinter root window
            robot: RobotController instance
            mocap_tracker: MotionCaptureTracker instance
        """
        self.root = root
        self.robot = robot
        self.mocap = mocap_tracker
        
        # Position tracking
        self.robot_x = 0.0  # Odometry position (cm)
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.target_x = None
        self.target_y = None
        
        # Trajectory history
        self.expected_trajectory = deque(maxlen=1000)  # (x, y, timestamp)
        self.actual_trajectory = deque(maxlen=1000)    # (x, y, timestamp)
        self.last_mocap_sample_time = 0.0
        self.mocap_sample_interval = 1.0  # Sample every 1 second
        
        # Canvas settings
        self.canvas_size = 500  # pixels per canvas
        self.world_size = 20.0  # cm (±10cm)
        self.scale = self.canvas_size / self.world_size
        
        # Colors
        self.expected_color = 'red'
        self.actual_color = 'green'
        
        # Setup GUI
        self.setup_ui()
        
        # Start update loops
        self.update_robot_telemetry()
        self.update_mocap_data()
        
    def setup_ui(self):
        """Create GUI elements."""
        self.root.title("Position Comparison - Odometry vs Motion Capture")
        self.root.geometry("1200x650")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, 
                         text="Robot Position Comparison: Expected vs Actual",
                         font=('Arial', 14, 'bold'))
        title.grid(row=0, column=0, columnspan=3, pady=10)
        
        # === LEFT SIDE: Control Window ===
        left_frame = ttk.LabelFrame(main_frame, text="CONTROL - Click to Navigate", 
                                   padding="10")
        left_frame.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.N, tk.W, tk.E))
        
        self.control_canvas = tk.Canvas(left_frame, width=self.canvas_size,
                                       height=self.canvas_size, bg='white',
                                       relief=tk.SUNKEN, borderwidth=2)
        self.control_canvas.pack()
        self.control_canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Control info
        control_info = ttk.Frame(left_frame)
        control_info.pack(pady=10)
        
        ttk.Label(control_info, text="Robot Position (Odometry):",
                 font=('Arial', 9, 'bold')).grid(row=0, column=0, columnspan=2)
        
        ttk.Label(control_info, text="X:").grid(row=1, column=0, sticky=tk.E)
        self.control_x_label = ttk.Label(control_info, text="0.00 cm")
        self.control_x_label.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_info, text="Y:").grid(row=2, column=0, sticky=tk.E)
        self.control_y_label = ttk.Label(control_info, text="0.00 cm")
        self.control_y_label.grid(row=2, column=1, sticky=tk.W, padx=5)
        
        # === CENTER: Control Buttons ===
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=1, column=1, padx=10, pady=10)
        
        ttk.Button(button_frame, text="Reset Position",
                  command=self.reset_position).pack(pady=5, fill=tk.X)
        ttk.Button(button_frame, text="Stop Robot",
                  command=self.stop_robot).pack(pady=5, fill=tk.X)
        ttk.Button(button_frame, text="Clear Trajectories",
                  command=self.clear_trajectories).pack(pady=5, fill=tk.X)
        
        ttk.Separator(button_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        ttk.Label(button_frame, text="Target:",
                 font=('Arial', 9, 'bold')).pack()
        self.target_label = ttk.Label(button_frame, text="None")
        self.target_label.pack()
        
        ttk.Separator(button_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        ttk.Label(button_frame, text="Motion Capture:",
                 font=('Arial', 9, 'bold')).pack()
        self.mocap_x_label = ttk.Label(button_frame, text="X: N/A")
        self.mocap_x_label.pack()
        self.mocap_y_label = ttk.Label(button_frame, text="Y: N/A")
        self.mocap_y_label.pack()
        
        # === RIGHT SIDE: Comparison Window ===
        right_frame = ttk.LabelFrame(main_frame, text="COMPARISON - Trajectory Analysis",
                                    padding="10")
        right_frame.grid(row=1, column=2, padx=10, pady=10, sticky=(tk.N, tk.W, tk.E))
        
        self.comparison_canvas = tk.Canvas(right_frame, width=self.canvas_size,
                                          height=self.canvas_size, bg='white',
                                          relief=tk.SUNKEN, borderwidth=2)
        self.comparison_canvas.pack()
        
        # Legend
        legend_frame = ttk.Frame(right_frame)
        legend_frame.pack(pady=10)
        
        # Expected trajectory legend
        expected_canvas = tk.Canvas(legend_frame, width=20, height=20, bg='white')
        expected_canvas.grid(row=0, column=0)
        expected_canvas.create_line(2, 10, 18, 10, fill='red', width=2)
        ttk.Label(legend_frame, text="Expected (Odometry)").grid(row=0, column=1, padx=5)
        
        # Actual trajectory legend
        actual_canvas = tk.Canvas(legend_frame, width=20, height=20, bg='white')
        actual_canvas.grid(row=1, column=0, pady=5)
        actual_canvas.create_oval(5, 5, 15, 15, fill='green', outline='darkgreen')
        ttk.Label(legend_frame, text="Actual (Motion Capture, 1s)").grid(row=1, column=1, padx=5)
        
        # Statistics
        stats_frame = ttk.LabelFrame(right_frame, text="Error Statistics", padding="5")
        stats_frame.pack(pady=5, fill=tk.X)
        
        self.error_label = ttk.Label(stats_frame, text="Position Error: N/A")
        self.error_label.pack()
        self.samples_label = ttk.Label(stats_frame, text="Samples: 0")
        self.samples_label.pack()
        
        # Status bar
        self.status_label = ttk.Label(main_frame, text="Ready",
                                     relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.grid(row=2, column=0, columnspan=3,
                              sticky=(tk.W, tk.E), pady=5)
        
        # Draw initial grids
        self.draw_control_grid()
        self.draw_comparison_grid()
        self.draw_robot_on_control()
        
    def draw_control_grid(self):
        """Draw grid on control canvas."""
        self.control_canvas.delete("grid")
        
        # Grid lines every 2cm
        for i in range(-10, 11, 2):
            x = self.world_to_canvas_x(i)
            self.control_canvas.create_line(x, 0, x, self.canvas_size,
                                           fill='lightgray', tags="grid")
            y = self.world_to_canvas_y(i)
            self.control_canvas.create_line(0, y, self.canvas_size, y,
                                           fill='lightgray', tags="grid")
        
        # Axes
        center_x = self.world_to_canvas_x(0)
        center_y = self.world_to_canvas_y(0)
        
        self.control_canvas.create_line(center_x, 0, center_x, self.canvas_size,
                                       fill='gray', width=2, tags="grid")
        self.control_canvas.create_line(0, center_y, self.canvas_size, center_y,
                                       fill='gray', width=2, tags="grid")
        
        # Labels
        for i in range(-10, 11, 5):
            if i != 0:
                x = self.world_to_canvas_x(i)
                y = self.world_to_canvas_y(i)
                self.control_canvas.create_text(x, self.canvas_size - 10,
                                               text=f"{i}", tags="grid", font=('Arial', 8))
                self.control_canvas.create_text(10, y, text=f"{i}",
                                               tags="grid", font=('Arial', 8))
        
        self.control_canvas.create_text(center_x + 15, center_y - 15,
                                       text="(0,0)", tags="grid",
                                       font=('Arial', 9, 'bold'))
    
    def draw_comparison_grid(self):
        """Draw grid on comparison canvas."""
        self.comparison_canvas.delete("grid")
        
        # Same as control grid
        for i in range(-10, 11, 2):
            x = self.world_to_canvas_x(i)
            self.comparison_canvas.create_line(x, 0, x, self.canvas_size,
                                              fill='lightgray', tags="grid")
            y = self.world_to_canvas_y(i)
            self.comparison_canvas.create_line(0, y, self.canvas_size, y,
                                              fill='lightgray', tags="grid")
        
        center_x = self.world_to_canvas_x(0)
        center_y = self.world_to_canvas_y(0)
        
        self.comparison_canvas.create_line(center_x, 0, center_x, self.canvas_size,
                                          fill='gray', width=2, tags="grid")
        self.comparison_canvas.create_line(0, center_y, self.canvas_size, center_y,
                                          fill='gray', width=2, tags="grid")
        
        for i in range(-10, 11, 5):
            if i != 0:
                x = self.world_to_canvas_x(i)
                y = self.world_to_canvas_y(i)
                self.comparison_canvas.create_text(x, self.canvas_size - 10,
                                                  text=f"{i}", tags="grid",
                                                  font=('Arial', 8))
                self.comparison_canvas.create_text(10, y, text=f"{i}",
                                                  tags="grid", font=('Arial', 8))
    
    def world_to_canvas_x(self, world_x):
        """Convert world X to canvas X."""
        return int((world_x + 10) * self.scale)
    
    def world_to_canvas_y(self, world_y):
        """Convert world Y to canvas Y (inverted)."""
        return int((10 - world_y) * self.scale)
    
    def canvas_to_world_x(self, canvas_x):
        """Convert canvas X to world X."""
        return (canvas_x / self.scale) - 10
    
    def canvas_to_world_y(self, canvas_y):
        """Convert canvas Y to world Y (inverted)."""
        return 10 - (canvas_y / self.scale)
    
    def draw_robot_on_control(self):
        """Draw robot on control canvas."""
        self.control_canvas.delete("robot")
        
        cx = self.world_to_canvas_x(self.robot_x)
        cy = self.world_to_canvas_y(self.robot_y)
        
        radius = 12
        self.control_canvas.create_oval(cx - radius, cy - radius,
                                       cx + radius, cy + radius,
                                       fill='blue', outline='darkblue', width=2,
                                       tags="robot")
        
        # Direction arrow
        arrow_length = radius + 8
        end_x = cx + arrow_length * math.cos(self.robot_theta)
        end_y = cy - arrow_length * math.sin(self.robot_theta)
        
        self.control_canvas.create_line(cx, cy, end_x, end_y,
                                       fill='yellow', width=3,
                                       arrow=tk.LAST, tags="robot")
    
    def draw_target_on_control(self):
        """Draw target marker on control canvas."""
        self.control_canvas.delete("target")
        
        if self.target_x is not None and self.target_y is not None:
            tx = self.world_to_canvas_x(self.target_x)
            ty = self.world_to_canvas_y(self.target_y)
            
            size = 10
            self.control_canvas.create_line(tx - size, ty, tx + size, ty,
                                           fill='red', width=2, tags="target")
            self.control_canvas.create_line(tx, ty - size, tx, ty + size,
                                           fill='red', width=2, tags="target")
            self.control_canvas.create_oval(tx - size, ty - size,
                                           tx + size, ty + size,
                                           outline='red', width=2, tags="target")
    
    def draw_trajectories(self):
        """Draw expected (red line) and actual (green dots) trajectories."""
        self.comparison_canvas.delete("trajectory")
        
        # Draw expected trajectory as RED LINE
        if len(self.expected_trajectory) > 1:
            points = []
            for x, y, _ in self.expected_trajectory:
                cx = self.world_to_canvas_x(x)
                cy = self.world_to_canvas_y(y)
                points.extend([cx, cy])
            
            if len(points) >= 4:  # At least 2 points
                self.comparison_canvas.create_line(*points, fill='red', width=2,
                                                  tags="trajectory", smooth=True)
        
        # Draw actual trajectory as GREEN DOTS (sampled every 1 second)
        for x, y, _ in self.actual_trajectory:
            cx = self.world_to_canvas_x(x)
            cy = self.world_to_canvas_y(y)
            radius = 4
            self.comparison_canvas.create_oval(cx - radius, cy - radius,
                                              cx + radius, cy + radius,
                                              fill='green', outline='darkgreen',
                                              width=1, tags="trajectory")
    
    def on_canvas_click(self, event):
        """Handle click on control canvas to set target."""
        world_x = self.canvas_to_world_x(event.x)
        world_y = self.canvas_to_world_y(event.y)
        
        world_x = round(world_x, 1)
        world_y = round(world_y, 1)
        
        if world_x < -10 or world_x > 10 or world_y < -10 or world_y > 10:
            messagebox.showwarning("Out of Bounds",
                                  "Target must be within ±10cm range")
            return
        
        self.target_x = world_x
        self.target_y = world_y
        
        self.target_label.config(text=f"({self.target_x:.1f}, {self.target_y:.1f}) cm")
        self.draw_target_on_control()
        
        self.send_goto_command(world_x, world_y)
    
    def send_goto_command(self, x, y):
        """Send goto command to robot."""
        try:
            self.status_label.config(text=f"Sending robot to ({x:.1f}, {y:.1f})...")
            self.root.update()
            
            command = f"g{x:.2f},{y:.2f}\n"
            self.robot.serial_conn.write(command.encode('utf-8'))
            
            self.status_label.config(text=f"Command sent: Go to ({x:.1f}, {y:.1f})")
            
        except Exception as e:
            messagebox.showerror("Communication Error", f"Failed to send: {e}")
            self.status_label.config(text="Error sending command")
    
    def reset_position(self):
        """Reset robot position to origin."""
        try:
            self.robot.send_command(b'z')
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_theta = 0.0
            self.target_x = None
            self.target_y = None
            
            self.control_x_label.config(text="0.00 cm")
            self.control_y_label.config(text="0.00 cm")
            self.target_label.config(text="None")
            
            self.draw_robot_on_control()
            self.control_canvas.delete("target")
            
            # Clear trajectories on reset
            self.clear_trajectories()
            
            self.status_label.config(text="Position reset to origin")
            
        except Exception as e:
            messagebox.showerror("Communication Error", f"Failed to reset: {e}")
    
    def stop_robot(self):
        """Emergency stop."""
        try:
            self.robot.stop()
            self.status_label.config(text="Robot stopped")
        except Exception as e:
            messagebox.showerror("Communication Error", f"Failed to stop: {e}")
    
    def clear_trajectories(self):
        """Clear trajectory history."""
        self.expected_trajectory.clear()
        self.actual_trajectory.clear()
        self.last_mocap_sample_time = 0.0
        self.draw_trajectories()
        self.samples_label.config(text="Samples: 0")
        self.error_label.config(text="Position Error: N/A")
        self.status_label.config(text="Trajectories cleared")
    
    def update_robot_telemetry(self):
        """Read robot odometry telemetry."""
        try:
            telemetry = self.robot.read_telemetry(timeout=0.05)
            
            if telemetry:
                if telemetry.startswith("POS:"):
                    self.parse_robot_position(telemetry)
                elif telemetry.startswith("NAV:"):
                    self.status_label.config(text=telemetry)
            
        except Exception as e:
            print(f"Telemetry error: {e}")
        
        self.root.after(100, self.update_robot_telemetry)
    
    def parse_robot_position(self, telemetry):
        """Parse robot position from telemetry."""
        try:
            parts = telemetry.split()
            
            for part in parts:
                if part.startswith("x="):
                    self.robot_x = float(part[2:])
                elif part.startswith("y="):
                    self.robot_y = float(part[2:])
                elif part.startswith("theta="):
                    self.robot_theta = float(part[6:])
            
            # Update display
            self.control_x_label.config(text=f"{self.robot_x:.2f} cm")
            self.control_y_label.config(text=f"{self.robot_y:.2f} cm")
            
            # Add to expected trajectory
            current_time = time.time()
            self.expected_trajectory.append((self.robot_x, self.robot_y, current_time))
            
            # Redraw
            self.draw_robot_on_control()
            self.draw_trajectories()
            
        except Exception as e:
            print(f"Error parsing position: {e}")
    
    def update_mocap_data(self):
        """Read motion capture data and update actual trajectory."""
        try:
            pos = self.mocap.get_current_position()
            
            if pos:
                x_cm, y_cm, timestamp = pos
                
                # Update display
                self.mocap_x_label.config(text=f"X: {x_cm:.2f} cm")
                self.mocap_y_label.config(text=f"Y: {y_cm:.2f} cm")
                
                # Sample at 1 second intervals for GREEN DOTS
                current_time = time.time()
                if current_time - self.last_mocap_sample_time >= self.mocap_sample_interval:
                    self.actual_trajectory.append((x_cm, y_cm, current_time))
                    self.last_mocap_sample_time = current_time
                    
                    # Update statistics
                    self.update_error_statistics()
                    self.draw_trajectories()
            else:
                self.mocap_x_label.config(text="X: N/A")
                self.mocap_y_label.config(text="Y: N/A")
            
        except Exception as e:
            print(f"MoCap error: {e}")
        
        self.root.after(100, self.update_mocap_data)
    
    def update_error_statistics(self):
        """Calculate and display position error statistics."""
        if len(self.actual_trajectory) == 0:
            return
        
        # Calculate current error (latest odometry vs latest mocap)
        if len(self.expected_trajectory) > 0 and len(self.actual_trajectory) > 0:
            expected_x, expected_y, _ = self.expected_trajectory[-1]
            actual_x, actual_y, _ = self.actual_trajectory[-1]
            
            error = math.sqrt((expected_x - actual_x)**2 + (expected_y - actual_y)**2)
            
            self.error_label.config(text=f"Position Error: {error:.2f} cm")
        
        self.samples_label.config(text=f"Samples: {len(self.actual_trajectory)}")


def main():
    """Main entry point."""
    print("Position Comparison System")
    print("=" * 60)
    print("Comparing Robot Odometry vs Motion Capture")
    print("=" * 60 + "\n")
    
    # Parse command line
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using COM port: {port}\n")
    else:
        print("No port specified, will auto-detect...\n")
    
    # Initialize robot controller
    robot = RobotController(connection_type='serial', port=port, baudrate=9600)
    
    print("Connecting to robot...")
    if not robot.connect():
        print("\nFailed to connect to robot!")
        print("Usage: python position_comparison.py [COM_PORT]")
        sys.exit(1)
    
    print("Robot connected!\n")
    
    # Initialize motion capture
    print("Initializing motion capture system...")
    mocap = MotionCaptureTracker(camera_id=1)
    
    if not mocap.start():
        print("\nFailed to start motion capture!")
        print("Check camera connection and calibration file.")
        robot.disconnect()
        sys.exit(1)
    
    print("Motion capture started!\n")
    
    # Create GUI
    root = tk.Tk()
    app = PositionComparisonGUI(root, robot, mocap)
    
    # Handle window close
    def on_closing():
        if messagebox.askokcancel("Quit", "Stop robot and quit?"):
            robot.stop()
            mocap.stop()
            robot.disconnect()
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Run GUI
    print("GUI launched!\n")
    print("LEFT: Click grid to navigate robot")
    print("RIGHT: Watch trajectory comparison")
    print("  RED LINE = Expected trajectory (odometry)")
    print("  GREEN DOTS = Actual position (motion capture, 1s intervals)\n")
    
    root.mainloop()
    
    # Cleanup
    mocap.stop()
    robot.disconnect()
    print("\nGoodbye!")


if __name__ == '__main__':
    main()
