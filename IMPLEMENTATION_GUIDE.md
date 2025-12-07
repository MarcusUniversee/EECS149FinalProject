# Complete Navigation System Implementation Guide

## System Overview

This document outlines the complete implementation of an autonomous robot navigation system that combines:
1. **Motion Capture** - AprilTag-based position tracking
2. **PC Controller** - User interface for setting target positions
3. **Bluetooth Control** - Wireless motor commands to the robot
4. **Closed-Loop Navigation** - Feedback control for accurate positioning

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     USER (PC Controller)                    │
│                  Sets target position (x, y)                │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│              Navigation Controller (New System)             │
│  • Receives target from user                                │
│  • Gets current position from Motion Capture                │
│  • Calculates required rotation and distance                │
│  • Sends motor commands via Bluetooth                       │
│  • Monitors position and corrects drift                     │
└─────┬────────────────────────────────────────────┬──────────┘
      │                                            │
      │ Motor Commands                             │ Position Query
      ▼                                            ▼
┌──────────────────────┐              ┌─────────────────────────┐
│   Bluetooth (HM-10)  │              │   Motion Capture        │
│  • Sends: speed L/R  │              │  • Detects AprilTag     │
│  • Receives: ack     │              │  • Calculates position  │
└──────────┬───────────┘              │  • Tracks orientation   │
           │                          └─────────────────────────┘
           ▼                                       │
┌─────────────────────┐                           │
│  Pololu 3pi+ Robot  │◄──────────────────────────┘
│  • Motors           │      Physical robot movement
│  • AprilTag         │      captured by camera
└─────────────────────┘
```

---

## Implementation Phases

### Phase 1: Verify Motion Capture with Relative Positioning ✅ COMPLETE

**Goal**: Verify that motion capture can track robot position relative to an initial origin.

**What We Built**:
- `relative_position_tracker.py` - Standalone tracking system
- Establishes initial position as (0, 0) origin
- Tracks relative X, Y position in cm
- Tracks relative orientation (yaw) in degrees
- Real-time visualization showing robot movement

**How to Test**:
```bash
cd motion_capture
python relative_position_tracker.py
```

**Expected Behavior**:
1. Program starts, waits for AprilTag detection
2. First detection sets origin at (0, 0)
3. Move robot by hand → see position update in real-time
4. Display shows: X, Y position in cm and yaw angle
5. Visual grid shows robot movement from origin

**Success Criteria**:
- ✓ Can detect AprilTag reliably
- ✓ Initial position becomes (0, 0)
- ✓ Moving robot updates position accurately
- ✓ Orientation (yaw) tracks robot rotation
- ✓ Position values are in cm (reasonable scale)

---

### Phase 2: Integrated Motion Capture + PC Controller

**Goal**: Combine GUI with motion capture to show both commanded and actual positions.

**What to Build**:
- `integrated_position_control.py` - Combined system
- Shows commanded target position from GUI clicks
- Shows actual robot position from motion capture
- Displays error/difference between commanded and actual
- Visual feedback with two markers (target vs actual)

**Key Features**:
```python
class IntegratedController:
    def __init__(self):
        self.target_x = None      # User's desired position
        self.target_y = None
        self.actual_x = 0.0       # Motion capture position
        self.actual_y = 0.0
        self.actual_yaw = 0.0     # Motion capture orientation
```

**Coordinate System Alignment**:
- Motion capture outputs in camera frame (X right, Y down, Z forward)
- PC controller expects robot frame (X forward, Y left)
- Must transform coordinates between systems

**How to Test**:
1. Start program with robot at origin
2. Click target position in GUI
3. Manually move robot to target
4. Verify motion capture shows correct position
5. Check that error decreases as robot approaches target

---

### Phase 3: Basic Turn-and-Drive Navigation

**Goal**: Robot autonomously navigates to target using turn-then-drive strategy.

**Algorithm**:
```
1. Get current position (x_current, y_current, θ_current) from motion capture
2. Get target position (x_target, y_target) from user
3. Calculate required heading:
   θ_target = atan2(y_target - y_current, x_target - x_current)
4. Calculate turn angle:
   θ_turn = θ_target - θ_current
5. Turn in place:
   - Set left_speed = +v, right_speed = -v (or opposite)
   - Monitor θ_current until |θ_current - θ_target| < threshold
6. Drive forward:
   - Set left_speed = v, right_speed = v
   - Monitor distance = sqrt((x_target-x_current)² + (y_target-y_current)²)
   - Stop when distance < threshold
```

**Rotation Control** (Differential Drive):
```python
def rotate_to_heading(target_heading, current_heading):
    """
    Rotate robot to face target heading.
    Differential drive: one wheel forward, one backward.
    """
    error = normalize_angle(target_heading - current_heading)
    
    # Choose rotation direction (shortest path)
    if abs(error) < HEADING_THRESHOLD:
        return True  # Done
    
    # Set motor speeds for rotation
    if error > 0:  # Turn counterclockwise
        left_speed = -TURN_SPEED
        right_speed = +TURN_SPEED
    else:  # Turn clockwise
        left_speed = +TURN_SPEED
        right_speed = -TURN_SPEED
    
    send_motor_command(left_speed, right_speed)
    return False
```

**Forward Drive**:
```python
def drive_to_target(target_x, target_y, current_x, current_y):
    """
    Drive forward to target position.
    """
    distance = math.sqrt((target_x - current_x)**2 + 
                        (target_y - current_y)**2)
    
    if distance < POSITION_THRESHOLD:
        send_motor_command(0, 0)  # Stop
        return True  # Done
    
    # Drive forward
    send_motor_command(DRIVE_SPEED, DRIVE_SPEED)
    return False
```

**What to Build**:
- `basic_navigation.py` - Simple turn-and-drive controller
- State machine: IDLE → TURNING → DRIVING → ARRIVED
- Uses motion capture for feedback
- Sends motor commands via Bluetooth

---

### Phase 4: Closed-Loop Navigation with Drift Correction

**Goal**: Add continuous feedback to correct for drift during driving.

**Problem**: 
- Robot may not drive perfectly straight
- Wheels may slip or have different speeds
- Need to continuously check heading while driving

**Solution - Continuous Heading Correction**:
```python
def drive_with_heading_correction(target_x, target_y, 
                                  current_x, current_y, 
                                  current_heading):
    """
    Drive toward target while correcting heading.
    """
    # Calculate desired heading
    dx = target_x - current_x
    dy = target_y - current_y
    desired_heading = math.atan2(dy, dx)
    
    # Calculate heading error
    heading_error = normalize_angle(desired_heading - current_heading)
    
    # Base speed
    base_speed = DRIVE_SPEED
    
    # PID control for heading correction
    # Positive error = need to turn left (increase right wheel, decrease left)
    correction = Kp * heading_error
    
    left_speed = base_speed - correction
    right_speed = base_speed + correction
    
    # Clamp speeds
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)
    
    send_motor_command(left_speed, right_speed)
```

**PID Controller**:
```python
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
    
    def update(self, error, dt):
        """Calculate control output."""
        # Proportional
        P = self.Kp * error
        
        # Integral
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative
        
        self.prev_error = error
        
        return P + I + D
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
```

**Enhanced State Machine**:
```
States:
- IDLE: Waiting for target
- INITIAL_TURN: Turn to face target
- DRIVING: Drive forward with heading correction
- ARRIVED: At target, stopped

Transitions:
IDLE → INITIAL_TURN: User clicks target
INITIAL_TURN → DRIVING: Heading error < threshold
DRIVING → ARRIVED: Distance to target < threshold
DRIVING → INITIAL_TURN: Heading error > large threshold (major drift)
ARRIVED → IDLE: User clicks new target
```

**What to Build**:
- `closed_loop_navigation.py` - Full navigation with feedback
- PID controller for heading correction
- State machine with drift handling
- Real-time visualization of trajectory
- Logging of position errors

---

## System Components Reference

### 1. Motion Capture (`relative_position_tracker.py`)

**Inputs**: 
- Camera feed
- Camera calibration data

**Outputs**:
- Robot position (x, y) in cm relative to origin
- Robot orientation (yaw) in degrees
- Detection confidence

**Key Functions**:
```python
tracker = RelativePositionTracker(tag_id=0)
rel_x, rel_y, rel_z, rel_yaw = tracker.update(position, orientation)
```

### 2. Bluetooth Motor Control (`test_command.py`)

**Current Interface**:
```python
# Command format: "left_speed right_speed\n"
# Speed range: -1.0 to +1.0
await client.write_gatt_char(CHAR_UUID, b'0.2 0.2\n', response=True)
```

**Needed Enhancements**:
```python
class RobotBluetooth:
    async def set_motor_speeds(self, left, right):
        """Set motor speeds (-1.0 to 1.0)"""
        command = f"{left:.2f} {right:.2f}\n"
        await self.client.write_gatt_char(CHAR_UUID, command.encode())
    
    async def stop(self):
        """Emergency stop"""
        await self.set_motor_speeds(0.0, 0.0)
```

### 3. PC Controller GUI (`position_control_gui.py`)

**Current Features**:
- 20cm × 20cm grid display
- Click to set target
- Shows commanded position

**Needed Enhancements**:
- Display actual position from motion capture
- Show error vector (target → actual)
- Display robot orientation
- Show navigation state (turning/driving/arrived)

---

## Coordinate System Transformations

### Camera Frame (Motion Capture)
```
     Z (forward, into scene)
    /
   /
  +-----→ X (right)
  |
  |
  ↓ Y (down)
```

### Robot Frame (Navigation)
```
    ↑ X (forward)
    |
    |
    +-----→ Y (left)
```

### Screen/GUI Frame
```
  ← X (left negative, right positive)
  |
  |
  ↓ Y (up positive, down negative)
```

**Transformation Example**:
```python
def camera_to_robot_frame(camera_x, camera_y, camera_yaw):
    """
    Convert camera frame to robot navigation frame.
    Assumes camera is mounted above, looking down.
    """
    # Camera X (right) → Robot Y (left)
    # Camera Y (down) → Robot -X (backward)
    robot_x = -camera_y
    robot_y = camera_x
    robot_yaw = camera_yaw  # Rotation is same
    return robot_x, robot_y, robot_yaw
```

---

## Configuration Parameters

### Motion Capture
```python
CAMERA_ID = 1                    # USB camera
IMAGE_RES = (640, 480)           # Lower = faster
TAG_FAMILY = 'tag16h5'           # Fast detection
TAG_SIZE = 0.023                 # 23mm physical size
MIN_DECISION_MARGIN = 50.0       # Quality threshold
```

### Navigation Control
```python
# Thresholds
POSITION_THRESHOLD = 1.0         # cm - target reached
HEADING_THRESHOLD = 5.0          # degrees - facing target

# Speeds (as fraction of max, -1.0 to 1.0)
TURN_SPEED = 0.3                 # Rotation speed
DRIVE_SPEED = 0.4                # Forward speed
MAX_SPEED = 0.8                  # Safety limit

# PID Gains (tune experimentally)
Kp_heading = 0.02                # Proportional gain
Ki_heading = 0.001               # Integral gain  
Kd_heading = 0.005               # Derivative gain
```

### Bluetooth
```python
ADDRESS = "B0:D2:78:32:EA:6C"    # HM-10 MAC address
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
```

---

## Testing Strategy

### Test 1: Motion Capture Accuracy
```
1. Place robot at origin
2. Mark positions at 5cm, 10cm, 15cm, 20cm
3. Move robot to each marked position
4. Record measured position from motion capture
5. Calculate error (should be < 1cm)
```

### Test 2: Rotation Accuracy
```
1. Place robot at origin
2. Command 90° turn
3. Measure actual rotation from motion capture
4. Repeat for 180°, 270°, -90°
5. Calculate error (should be < 5°)
```

### Test 3: Straight Line Drive
```
1. Place robot at origin, facing 0°
2. Command forward at constant speed
3. Monitor heading during drive
4. Measure final position and orientation
5. Check for drift (should stay within ±10° heading)
```

### Test 4: Navigate to Waypoints
```
Waypoints: (0,0) → (5,0) → (5,5) → (0,5) → (0,0)
1. Start at origin
2. Navigate to each waypoint
3. Record position error at each arrival
4. Measure total time
5. Success if all errors < 2cm
```

### Test 5: Drift Correction
```
1. Place obstacle to force drift
2. Command forward motion
3. Manually push robot sideways during motion
4. Verify controller corrects back to heading
5. Check final position error
```

---

## Troubleshooting

### Motion Capture Issues

**Problem**: Tag not detected
- Check lighting (need good, even lighting)
- Verify tag is not occluded
- Try adjusting MIN_DECISION_MARGIN
- Check camera focus

**Problem**: Jittery position readings
- Increase smoothing window size
- Check for camera motion blur
- Reduce frame rate if CPU overloaded
- Verify camera is stable/mounted

**Problem**: Wrong coordinate scale
- Verify TAG_SIZE matches physical tag (23mm = 0.023m)
- Check camera calibration is correct
- Measure actual distance and compare to reported

### Bluetooth Control Issues

**Problem**: Commands not received
- Check Bluetooth pairing
- Verify MAC address is correct
- Test with simple test_command.py first
- Check baud rate matches (9600)

**Problem**: Robot moves erratically
- Verify command format: "left right\n"
- Check speed values are in range [-1.0, 1.0]
- Add delays between commands
- Check for buffer overflow on robot

### Navigation Issues

**Problem**: Robot overshoots target
- Reduce DRIVE_SPEED
- Increase POSITION_THRESHOLD
- Add deceleration as approaching target
- Check for latency in motion capture

**Problem**: Robot can't turn accurately
- Increase TURN_SPEED for faster response
- Decrease HEADING_THRESHOLD for tighter tolerance
- Check wheel slippage on surface
- Verify differential drive math

**Problem**: Drift not corrected
- Increase Kp gain for stronger correction
- Check heading calculation (angle wrapping)
- Verify motion capture provides good yaw data
- Reduce DRIVE_SPEED for more time to correct

---

## Next Steps

1. **Test Phase 1** ✅
   - Run `relative_position_tracker.py`
   - Verify motion capture works
   - Measure accuracy by hand

2. **Implement Phase 2**
   - Create integrated GUI + motion capture
   - Test coordinate transformations
   - Verify position display accuracy

3. **Implement Phase 3**
   - Create basic turn-and-drive controller
   - Test rotation in place
   - Test straight-line driving

4. **Implement Phase 4**
   - Add PID heading controller
   - Test drift correction
   - Tune PID gains experimentally

5. **Full System Test**
   - Run complete navigation demos
   - Measure position accuracy
   - Test robustness to disturbances

---

## Code Organization

```
EECS149FinalProject/
├── motion_capture/
│   ├── apriltag_tracker.py              # Original full tracker
│   ├── relative_position_tracker.py     # Phase 1 - Relative tracking ✅
│   └── camera_calibration.yaml
├── pc_controller/
│   ├── position_control_gui.py          # Original GUI
│   └── robot_controller.py              # Bluetooth interface
├── navigation/                           # NEW FOLDER
│   ├── integrated_control.py            # Phase 2 - GUI + Motion Capture
│   ├── basic_navigation.py              # Phase 3 - Turn and drive
│   ├── closed_loop_navigation.py        # Phase 4 - Full feedback control
│   ├── pid_controller.py                # PID implementation
│   └── coordinate_transforms.py         # Frame transformations
├── test_command.py                      # Bluetooth test
└── IMPLEMENTATION_GUIDE.md              # This file
```

---

## Summary

You now have:

1. **Complete implementation plan** with 4 phases
2. **Working Phase 1** - Motion capture relative position tracker
3. **Detailed algorithms** for each phase
4. **Testing procedures** to verify each component
5. **Troubleshooting guide** for common issues

Start by testing `relative_position_tracker.py` to verify motion capture works with your setup. Once that's working reliably, we can move on to implementing the integrated navigation system.
