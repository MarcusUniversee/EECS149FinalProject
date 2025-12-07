"""
relative_position_tracker.py

Motion capture system that tracks robot position relative to its initial position.
This allows us to treat the starting position as (0, 0) and track movement from there.

The system:
1. Detects the AprilTag on the robot
2. Records the initial position as the origin (0, 0, 0°)
3. Continuously tracks relative position and orientation
4. Displays position on screen in real-time

Coordinate System:
- X: horizontal (right is positive)
- Y: vertical (up is positive)
- Theta: rotation (counterclockwise from +X axis is positive)
"""

import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector
import math
import time
import os
from collections import deque
from position_tracker import RelativePositionTracker

# ==================== CONFIGURATION ====================
# Camera settings
CAMERA_ID = 1
IMAGE_RES = (640, 480)
# Get path to calibration file relative to this script
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), 'camera_calibration.yaml')

# AprilTag settings
TAG_FAMILY = 'tag36h11'
TAG_SIZE = 0.0475  # 47.5mm
MIN_DECISION_MARGIN = 50.0

# Robot tag ID 
ROBOT_TAG_ID = 0

# Display settings
WINDOW_NAME = 'Relative Position Tracker'
DISPLAY_SCALE = 30  # pixels per cm for visualization
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
        print("Please run camera_calibration.py first.")
        exit(1)
    except Exception as e:
        print(f"Error loading calibration: {e}")
        exit(1)


def initialize_camera(camera_id, resolution):
    """Initialize camera with optimized settings."""
    # Try DirectShow first (Windows)
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        # Fallback to default
        cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    
    # Optimize for speed
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


# NOTE: RelativePositionTracker class is now imported from position_tracker.py
# This avoids code duplication and makes it reusable by navigation systems


def _OLD_RelativePositionTracker_MOVED_TO_position_tracker_py():
    """
    Tracks robot position relative to initial position.
    """
    
    def __init__(self, tag_id):
        """
        Initialize tracker.
        
        Args:
            tag_id: AprilTag ID to track
        """
        self.tag_id = tag_id
        self.initial_position = None  # (x, y, z) in meters
        self.initial_orientation = None  # (roll, pitch, yaw) in degrees
        self.is_initialized = False
        
        # Current measurements (relative)
        self.rel_x = 0.0  # cm
        self.rel_y = 0.0  # cm
        self.rel_z = 0.0  # cm
        self.rel_yaw = 0.0  # degrees
        
        # Smoothing filter
        self.position_history = deque(maxlen=5)
        self.yaw_history = deque(maxlen=5)
        
    def initialize(self, position, orientation):
        """
        Set initial reference position.
        
        Args:
            position: (x, y, z) tuple in meters
            orientation: (roll, pitch, yaw) tuple in degrees
        """
        self.initial_position = position
        self.initial_orientation = orientation
        self.is_initialized = True
        
        print("\n" + "="*70)
        print("INITIAL POSITION SET (Origin Established)")
        print("="*70)
        print(f"Reference Position (m): X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f}")
        print(f"Reference Orientation (deg): Yaw={orientation[2]:.1f}°")
        print("="*70)
        print("\nNow tracking relative movements from this origin...")
        print("Move the robot by hand to see relative position updates.\n")
    
    def update(self, position, orientation):
        """
        Update with new measurement and calculate relative position.
        
        Args:
            position: (x, y, z) tuple in meters (camera frame)
            orientation: (roll, pitch, yaw) tuple in degrees
        
        Returns:
            tuple: (rel_x, rel_y, rel_z, rel_yaw) relative position in cm and degrees
        """
        if not self.is_initialized:
            self.initialize(position, orientation)
            return (0.0, 0.0, 0.0, 0.0)
        
        # Calculate relative position in camera frame (meters)
        dx = position[0] - self.initial_position[0]
        dy = position[1] - self.initial_position[1]
        dz = position[2] - self.initial_position[2]
        
        # Convert to cm
        self.rel_x = dx * 100
        self.rel_y = dy * 100
        self.rel_z = dz * 100
        
        # Calculate relative yaw (orientation)
        dyaw = orientation[2] - self.initial_orientation[2]
        
        # Normalize angle to [-180, 180]
        while dyaw > 180:
            dyaw -= 360
        while dyaw < -180:
            dyaw += 360
        
        self.rel_yaw = dyaw
        
        # Apply smoothing
        self.position_history.append((self.rel_x, self.rel_y, self.rel_z))
        self.yaw_history.append(self.rel_yaw)
        
        # Return smoothed values
        smoothed_x = sum(p[0] for p in self.position_history) / len(self.position_history)
        smoothed_y = sum(p[1] for p in self.position_history) / len(self.position_history)
        smoothed_z = sum(p[2] for p in self.position_history) / len(self.position_history)
        smoothed_yaw = sum(self.yaw_history) / len(self.yaw_history)
        
        return (smoothed_x, smoothed_y, smoothed_z, smoothed_yaw)
    
    def reset(self):
        """Reset tracker to uninitialized state."""
        self.initial_position = None
        self.initial_orientation = None
        self.is_initialized = False
        self.position_history.clear()
        self.yaw_history.clear()
        print("\nTracker reset. Move robot to set new origin.\n")


def create_visualization_display(rel_x, rel_y, rel_yaw, tracking_active):
    """
    Create a visual display showing robot position.
    
    Args:
        rel_x: Relative X position in cm
        rel_y: Relative Y position in cm
        rel_yaw: Relative yaw in degrees
        tracking_active: Whether robot is currently detected
    
    Returns:
        numpy.ndarray: Display image
    """
    # Create display (600x600 pixels, 20cm x 20cm range)
    display_size = 600
    display = np.ones((display_size, display_size, 3), dtype=np.uint8) * 255
    
    # Draw grid (2cm spacing)
    grid_color = (200, 200, 200)
    for i in range(-10, 11, 2):
        # Vertical lines
        x = int((i + 10) * DISPLAY_SCALE)
        cv2.line(display, (x, 0), (x, display_size), grid_color, 1)
        
        # Horizontal lines
        y = int((10 - i) * DISPLAY_SCALE)
        cv2.line(display, (0, y), (display_size, y), grid_color, 1)
    
    # Draw axes
    center = display_size // 2
    cv2.line(display, (center, 0), (center, display_size), (150, 150, 150), 2)
    cv2.line(display, (0, center), (display_size, center), (150, 150, 150), 2)
    
    # Draw axis labels
    for i in range(-10, 11, 5):
        if i != 0:
            x_pos = int((i + 10) * DISPLAY_SCALE)
            y_pos = int((10 - i) * DISPLAY_SCALE)
            cv2.putText(display, f"{i}", (x_pos - 10, display_size - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
            cv2.putText(display, f"{i}", (5, y_pos + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    
    # Draw origin marker
    cv2.circle(display, (center, center), 5, (0, 0, 255), -1)
    cv2.putText(display, "Origin", (center + 10, center - 10), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # Draw robot if tracking
    if tracking_active:
        # Convert position to display coordinates
        robot_x = int((rel_x + 10) * DISPLAY_SCALE)
        robot_y = int((10 - rel_y) * DISPLAY_SCALE)
        
        # Check if in bounds
        if 0 <= robot_x < display_size and 0 <= robot_y < display_size:
            # Draw robot body
            cv2.circle(display, (robot_x, robot_y), 15, (0, 255, 0), -1)
            cv2.circle(display, (robot_x, robot_y), 15, (0, 150, 0), 2)
            
            # Draw heading indicator
            yaw_rad = math.radians(rel_yaw)
            arrow_len = 25
            end_x = int(robot_x + arrow_len * math.cos(yaw_rad))
            end_y = int(robot_y - arrow_len * math.sin(yaw_rad))
            cv2.arrowedLine(display, (robot_x, robot_y), (end_x, end_y), 
                          (255, 0, 0), 2, tipLength=0.3)
            
            # Draw trail line from origin
            cv2.line(display, (center, center), (robot_x, robot_y), 
                    (0, 200, 0), 1, cv2.LINE_AA)
    
    # Draw status text
    y_offset = 30
    line_height = 30
    
    status_color = (0, 180, 0) if tracking_active else (0, 0, 180)
    status_text = "TRACKING ACTIVE" if tracking_active else "WAITING FOR TAG..."
    
    cv2.putText(display, status_text, (10, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    
    if tracking_active:
        y_offset += line_height
        cv2.putText(display, f"X: {rel_x:+7.2f} cm", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        y_offset += line_height
        cv2.putText(display, f"Y: {rel_y:+7.2f} cm", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        y_offset += line_height
        cv2.putText(display, f"Yaw: {rel_yaw:+7.1f} deg", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Distance from origin
        distance = math.sqrt(rel_x**2 + rel_y**2)
        y_offset += line_height
        cv2.putText(display, f"Distance: {distance:.2f} cm", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    # Instructions at bottom
    instructions = [
        "Controls:",
        "  R - Reset origin",
        "  Q - Quit"
    ]
    
    y_offset = display_size - 60
    for instruction in instructions:
        cv2.putText(display, instruction, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        y_offset += 20
    
    return display


class PerformanceMonitor:
    """Monitor tracking performance."""
    
    def __init__(self, window_size=30):
        self.frame_times = deque(maxlen=window_size)
        self.last_time = time.time()
        
    def update(self):
        """Update timing."""
        current_time = time.time()
        self.frame_times.append(current_time - self.last_time)
        self.last_time = current_time
    
    def get_fps(self):
        """Get average FPS."""
        if not self.frame_times:
            return 0.0
        avg_time = sum(self.frame_times) / len(self.frame_times)
        return 1.0 / avg_time if avg_time > 0 else 0.0


def main():
    """Main tracking loop."""
    print("\n" + "="*70)
    print("RELATIVE POSITION TRACKER")
    print("="*70)
    print("\nThis program tracks the robot's position relative to where it starts.")
    print("The first detected position becomes the origin (0, 0).")
    print("\nPlace the robot with AprilTag visible to the camera.")
    print("Then move it by hand to see relative position tracking.\n")
    
    # Load calibration
    camera_matrix, dist_coeffs = load_camera_calibration(CALIBRATION_FILE)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    
    # Initialize camera
    cap = initialize_camera(CAMERA_ID, IMAGE_RES)
    if cap is None:
        return
    
    # Initialize detector
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    
    print("✓ AprilTag detector initialized")
    
    # Initialize tracker
    tracker = RelativePositionTracker(ROBOT_TAG_ID)
    
    # Performance monitor
    perf = PerformanceMonitor()
    
    print("\n" + "="*70)
    print("READY - Waiting for robot tag...")
    print("="*70)
    print("\nControls:")
    print("  R - Reset origin to current position")
    print("  Q or ESC - Quit")
    print("="*70 + "\n")
    
    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Detect tags
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[fx, fy, cx, cy],
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
            tracking_active = False
            rel_x, rel_y, rel_z, rel_yaw = 0, 0, 0, 0
            
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
                rel_x, rel_y, rel_z, rel_yaw = tracker.update(position, orientation)
                tracking_active = True
                
                # Print to console
                if tracker.is_initialized:
                    print(f"Position: X={rel_x:+7.2f}cm  Y={rel_y:+7.2f}cm  Yaw={rel_yaw:+7.1f}°  "
                          f"FPS={perf.get_fps():.1f}", end='\r')
            
            # Create visualization
            viz = create_visualization_display(rel_x, rel_y, rel_yaw, tracking_active)
            
            # Show display
            cv2.imshow(WINDOW_NAME, viz)
            
            # Update performance
            perf.update()
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # Q or ESC
                print("\n\nExiting...")
                break
            elif key == ord('r'):  # R - reset
                tracker.reset()
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera released. Goodbye!\n")


if __name__ == "__main__":
    main()
