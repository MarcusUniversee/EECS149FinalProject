"""
integrated_position_control.py

Integrated system combining:
1. Motion capture (AprilTag tracking) for actual robot position
2. PC Controller GUI for visual display and target selection
3. Real-time position feedback

This displays the robot's actual position from the camera on the GUI grid.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import sys
import os
import threading
import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from motion_capture.position_tracker import RelativePositionTracker


# ==================== CONFIGURATION ====================
# Camera settings
CAMERA_ID = 1
IMAGE_RES = (640, 480)
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '..', 'motion_capture', 'camera_calibration.yaml')

# AprilTag settings
TAG_FAMILY = 'tag36h11'
TAG_SIZE = 0.0475  # 47.5mm
MIN_DECISION_MARGIN = 50.0
ROBOT_TAG_ID = 0

# GUI settings
CANVAS_SIZE = 600  # pixels
WORLD_SIZE = 40.0  # cm (±20cm range)
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


class MotionCaptureThread(threading.Thread):
    """
    Background thread for motion capture.
    Continuously tracks robot position and updates GUI.
    """
    
    def __init__(self, gui):
        """
        Initialize motion capture thread.
        
        Args:
            gui: IntegratedPositionGUI instance to update
        """
        super().__init__(daemon=True)
        self.gui = gui
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
        """Main tracking loop."""
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
                    
                    # Update GUI (thread-safe)
                    if self.tracker.is_initialized:
                        self.gui.update_robot_position(rel_x, rel_y, rel_yaw)
                
            except Exception as e:
                print(f"Motion capture error: {e}")
    
    def stop(self):
        """Stop tracking."""
        self.running = False
        if self.cap:
            self.cap.release()


class IntegratedPositionGUI:
    """
    GUI that displays robot position from motion capture.
    """
    
    def __init__(self, root):
        """
        Initialize GUI.
        
        Args:
            root: Tkinter root window
        """
        self.root = root
        
        # Position tracking (from motion capture)
        self.robot_x = 0.0  # cm
        self.robot_y = 0.0  # cm
        self.robot_yaw = 0.0  # degrees
        self.target_x = None
        self.target_y = None
        self.tracking_active = False
        
        # Canvas settings (40cm x 40cm range, scaled to pixels)
        self.canvas_size = CANVAS_SIZE
        self.world_size = WORLD_SIZE
        self.scale = self.canvas_size / self.world_size  # pixels per cm
        
        # Setup GUI
        self.setup_ui()
        
        # Start motion capture thread
        self.motion_capture = MotionCaptureThread(self)
        if self.motion_capture.initialize():
            self.motion_capture.start()
            print("✓ Motion capture thread started")
        else:
            print("⚠ Warning: Motion capture not available")
        
        # Start periodic update
        self.update_display()
        
    def setup_ui(self):
        """Create GUI elements."""
        self.root.title("Robot Position Control with Motion Capture")
        self.root.geometry("850x700")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, text="Robot Position Controller (Motion Capture)", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Instructions
        instructions = ttk.Label(main_frame, 
                                text="Live tracking from camera - 40cm × 40cm workspace",
                                font=('Arial', 10))
        instructions.grid(row=1, column=0, columnspan=2, pady=5)
        
        # Canvas for position display
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=2, column=0, padx=10, pady=10)
        
        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_size, 
                               height=self.canvas_size, bg='white', 
                               relief=tk.SUNKEN, borderwidth=2)
        self.canvas.pack()
        
        # Bind click event
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Robot Status", padding="10")
        control_frame.grid(row=2, column=1, padx=10, pady=10, sticky=(tk.N, tk.W, tk.E))
        
        # Tracking status
        ttk.Label(control_frame, text="Tracking Status:", 
                 font=('Arial', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)
        
        self.tracking_label = ttk.Label(control_frame, text="Waiting for tag...", 
                                        font=('Arial', 10), foreground='orange')
        self.tracking_label.grid(row=1, column=0, columnspan=2, pady=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Position display
        ttk.Label(control_frame, text="Current Position:", 
                 font=('Arial', 10, 'bold')).grid(row=3, column=0, columnspan=2, pady=5)
        
        ttk.Label(control_frame, text="X:").grid(row=4, column=0, sticky=tk.E)
        self.x_label = ttk.Label(control_frame, text="0.00 cm", 
                                font=('Arial', 10))
        self.x_label.grid(row=4, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Y:").grid(row=5, column=0, sticky=tk.E)
        self.y_label = ttk.Label(control_frame, text="0.00 cm", 
                                font=('Arial', 10))
        self.y_label.grid(row=5, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Yaw:").grid(row=6, column=0, sticky=tk.E)
        self.yaw_label = ttk.Label(control_frame, text="0.00°", 
                                   font=('Arial', 10))
        self.yaw_label.grid(row=6, column=1, sticky=tk.W, padx=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Target display
        ttk.Label(control_frame, text="Target Position:", 
                 font=('Arial', 10, 'bold')).grid(row=8, column=0, columnspan=2, pady=5)
        
        self.target_label = ttk.Label(control_frame, text="None", 
                                      font=('Arial', 10))
        self.target_label.grid(row=9, column=0, columnspan=2, pady=5)
        
        # Distance to target
        ttk.Label(control_frame, text="Distance to Target:", 
                 font=('Arial', 9)).grid(row=10, column=0, columnspan=2, pady=5)
        
        self.distance_label = ttk.Label(control_frame, text="--", 
                                       font=('Arial', 10, 'bold'))
        self.distance_label.grid(row=11, column=0, columnspan=2, pady=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=12, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Buttons
        ttk.Button(control_frame, text="Reset Origin", 
                  command=self.reset_origin).grid(row=13, column=0, columnspan=2, 
                                                  pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(control_frame, text="Clear Target", 
                  command=self.clear_target).grid(row=14, column=0, columnspan=2, 
                                                 pady=5, sticky=(tk.W, tk.E))
        
        # Status bar
        self.status_label = ttk.Label(main_frame, text="Ready - Motion capture active", 
                                     relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.grid(row=3, column=0, columnspan=2, 
                              sticky=(tk.W, tk.E), pady=5)
        
        # Draw initial grid
        self.draw_grid()
        self.draw_robot()
        
    def draw_grid(self):
        """Draw coordinate grid."""
        self.canvas.delete("grid")
        
        # Grid lines every 2cm
        for i in range(-20, 21, 2):
            # Vertical lines
            x = self.world_to_canvas_x(i)
            self.canvas.create_line(x, 0, x, self.canvas_size, 
                                   fill='lightgray', tags="grid")
            
            # Horizontal lines
            y = self.world_to_canvas_y(i)
            self.canvas.create_line(0, y, self.canvas_size, y, 
                                   fill='lightgray', tags="grid")
        
        # Axes (darker)
        center_x = self.world_to_canvas_x(0)
        center_y = self.world_to_canvas_y(0)
        
        self.canvas.create_line(center_x, 0, center_x, self.canvas_size, 
                               fill='gray', width=2, tags="grid")
        self.canvas.create_line(0, center_y, self.canvas_size, center_y, 
                               fill='gray', width=2, tags="grid")
        
        # Labels
        for i in range(-20, 21, 5):
            if i != 0:
                x = self.world_to_canvas_x(i)
                y = self.world_to_canvas_y(i)
                self.canvas.create_text(x, self.canvas_size - 10, 
                                       text=f"{i}", tags="grid")
                self.canvas.create_text(10, y, text=f"{i}", tags="grid")
        
        # Origin label
        self.canvas.create_text(center_x + 15, center_y - 15, 
                               text="(0,0)", tags="grid", font=('Arial', 10, 'bold'))
    
    def world_to_canvas_x(self, world_x):
        """Convert world X coordinate to canvas X."""
        return int((world_x + 20) * self.scale)
    
    def world_to_canvas_y(self, world_y):
        """Convert world Y coordinate to canvas Y (inverted)."""
        return int((20 - world_y) * self.scale)
    
    def canvas_to_world_x(self, canvas_x):
        """Convert canvas X to world X coordinate."""
        return (canvas_x / self.scale) - 20
    
    def canvas_to_world_y(self, canvas_y):
        """Convert canvas Y to world Y coordinate (inverted)."""
        return 20 - (canvas_y / self.scale)
    
    def draw_robot(self):
        """Draw robot at current position."""
        self.canvas.delete("robot")
        
        if not self.tracking_active:
            return
        
        # Robot position
        cx = self.world_to_canvas_x(self.robot_x)
        cy = self.world_to_canvas_y(self.robot_y)
        
        # Robot body (circle)
        radius = 15
        self.canvas.create_oval(cx - radius, cy - radius, 
                               cx + radius, cy + radius,
                               fill='blue', outline='darkblue', width=2, 
                               tags="robot")
        
        # Direction indicator (line showing heading)
        # Convert yaw from degrees to radians
        yaw_rad = math.radians(self.robot_yaw)
        arrow_length = radius + 10
        end_x = cx + arrow_length * math.cos(yaw_rad)
        end_y = cy - arrow_length * math.sin(yaw_rad)  # Negative for canvas coords
        
        self.canvas.create_line(cx, cy, end_x, end_y, 
                               fill='yellow', width=3, 
                               arrow=tk.LAST, tags="robot")
        
        # Position label
        self.canvas.create_text(cx, cy + radius + 15, 
                               text=f"({self.robot_x:.1f}, {self.robot_y:.1f})",
                               tags="robot", font=('Arial', 9))
    
    def draw_target(self):
        """Draw target position."""
        self.canvas.delete("target")
        
        if self.target_x is not None and self.target_y is not None:
            tx = self.world_to_canvas_x(self.target_x)
            ty = self.world_to_canvas_y(self.target_y)
            
            # Target marker (crosshair)
            size = 10
            self.canvas.create_line(tx - size, ty, tx + size, ty, 
                                   fill='red', width=2, tags="target")
            self.canvas.create_line(tx, ty - size, tx, ty + size, 
                                   fill='red', width=2, tags="target")
            
            # Target circle
            self.canvas.create_oval(tx - size, ty - size, 
                                   tx + size, ty + size,
                                   outline='red', width=2, tags="target")
            
            # Target label
            self.canvas.create_text(tx, ty - size - 15, 
                                   text=f"Target: ({self.target_x:.1f}, {self.target_y:.1f})",
                                   tags="target", fill='red', font=('Arial', 9, 'bold'))
            
            # Draw line from robot to target if tracking
            if self.tracking_active:
                rx = self.world_to_canvas_x(self.robot_x)
                ry = self.world_to_canvas_y(self.robot_y)
                self.canvas.create_line(rx, ry, tx, ty, 
                                       fill='green', width=2, dash=(5, 3), tags="target")
    
    def on_canvas_click(self, event):
        """Handle canvas click to set target."""
        # Convert click position to world coordinates
        world_x = self.canvas_to_world_x(event.x)
        world_y = self.canvas_to_world_y(event.y)
        
        # Round to 1 decimal place
        world_x = round(world_x, 1)
        world_y = round(world_y, 1)
        
        # Check bounds
        if world_x < -20 or world_x > 20 or world_y < -20 or world_y > 20:
            messagebox.showwarning("Out of Bounds", 
                                  "Target position must be within ±20cm range")
            return
        
        # Set target
        self.target_x = world_x
        self.target_y = world_y
        
        # Update display
        self.target_label.config(text=f"({self.target_x:.1f}, {self.target_y:.1f}) cm")
        self.draw_target()
        self.update_distance()
        
        self.status_label.config(text=f"Target set at ({world_x:.1f}, {world_y:.1f})")
    
    def update_robot_position(self, x, y, yaw):
        """
        Update robot position from motion capture (thread-safe).
        
        Args:
            x: X position in cm
            y: Y position in cm
            yaw: Yaw angle in degrees
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.tracking_active = True
    
    def update_display(self):
        """Periodically update display."""
        # Update position labels
        if self.tracking_active:
            self.x_label.config(text=f"{self.robot_x:+.2f} cm")
            self.y_label.config(text=f"{self.robot_y:+.2f} cm")
            self.yaw_label.config(text=f"{self.robot_yaw:+.1f}°")
            self.tracking_label.config(text="✓ Tracking Active", foreground='green')
        else:
            self.tracking_label.config(text="Waiting for tag...", foreground='orange')
        
        # Redraw robot and target
        self.draw_robot()
        self.draw_target()
        
        # Update distance to target
        self.update_distance()
        
        # Schedule next update (30 Hz)
        self.root.after(33, self.update_display)
    
    def update_distance(self):
        """Update distance to target display."""
        if self.tracking_active and self.target_x is not None and self.target_y is not None:
            dx = self.target_x - self.robot_x
            dy = self.target_y - self.robot_y
            distance = math.sqrt(dx**2 + dy**2)
            self.distance_label.config(text=f"{distance:.2f} cm")
        else:
            self.distance_label.config(text="--")
    
    def reset_origin(self):
        """Reset origin to current robot position."""
        if hasattr(self.motion_capture, 'tracker') and self.motion_capture.tracker:
            self.motion_capture.tracker.reset()
            self.status_label.config(text="Origin reset - Move robot to set new origin")
    
    def clear_target(self):
        """Clear target position."""
        self.target_x = None
        self.target_y = None
        self.target_label.config(text="None")
        self.canvas.delete("target")
        self.status_label.config(text="Target cleared")


def main():
    """Main entry point."""
    print("\n" + "="*70)
    print("INTEGRATED POSITION CONTROL WITH MOTION CAPTURE")
    print("="*70)
    print("\nThis system displays the robot's actual position from camera tracking.")
    print("Click anywhere on the grid to set a target position.\n")
    
    # Create GUI
    root = tk.Tk()
    app = IntegratedPositionGUI(root)
    
    # Handle window close
    def on_closing():
        if messagebox.askokcancel("Quit", "Stop tracking and quit?"):
            if app.motion_capture:
                app.motion_capture.stop()
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Run GUI
    print("GUI launched. Robot position will update automatically from camera.\n")
    root.mainloop()
    
    print("\nGoodbye!")


if __name__ == '__main__':
    main()
