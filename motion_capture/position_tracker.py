"""
position_tracker.py

Reusable RelativePositionTracker class for navigation.
This module contains ONLY the relative tracking logic,
designed to be imported by navigation systems.

Usage:
    from motion_capture.position_tracker import RelativePositionTracker
    
    tracker = RelativePositionTracker(tag_id=0)
    rel_x, rel_y, rel_z, rel_yaw = tracker.update(position, orientation)
"""

from collections import deque
import math


class RelativePositionTracker:
    """
    Tracks robot position relative to initial position.
    
    Converts absolute camera coordinates to relative robot coordinates,
    making navigation easier by treating the start position as origin (0, 0).
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
        
        # Smoothing filter (increased for stability)
        self.position_history = deque(maxlen=15)
        self.yaw_history = deque(maxlen=15)
        
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
    
    def update(self, position, orientation):
        """
        Update with new measurement and calculate relative position.
        
        Args:
            position: (x, y, z) tuple in meters (camera frame)
            orientation: (roll, pitch, yaw) tuple in degrees
        
        Returns:
            tuple: (rel_x, rel_y, rel_z, rel_yaw) 
                   Position in cm, orientation in degrees, all relative to initial
        """
        if not self.is_initialized:
            self.initialize(position, orientation)
            return (0.0, 0.0, 0.0, 0.0)
        
        # Calculate relative position in camera frame (meters)
        dx = position[0] - self.initial_position[0]
        dy = position[1] - self.initial_position[1]
        dz = position[2] - self.initial_position[2]
        
        # Convert to cm and flip X coordinate
        self.rel_x = -dx * 100  # Flipped
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
    
    def get_distance_from_origin(self):
        """
        Calculate 2D distance from initial position.
        
        Returns:
            float: Distance in cm
        """
        return math.sqrt(self.rel_x**2 + self.rel_y**2)
    
    def reset(self):
        """Reset tracker to uninitialized state."""
        self.initial_position = None
        self.initial_orientation = None
        self.is_initialized = False
        self.position_history.clear()
        self.yaw_history.clear()
        print("\nTracker reset. Next detection will set new origin.\n")
    
    def set_origin_manually(self, position, orientation):
        """
        Manually set origin without automatic initialization.
        Useful for setting origin before robot is detected.
        
        Args:
            position: (x, y, z) tuple in meters
            orientation: (roll, pitch, yaw) tuple in degrees
        """
        self.initialize(position, orientation)
