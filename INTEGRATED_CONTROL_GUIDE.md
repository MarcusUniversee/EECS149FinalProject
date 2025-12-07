# Integrated Position Control - Quick Guide

## What It Does

The integrated system combines:
- **Motion Capture**: AprilTag camera tracking for real robot position
- **GUI Display**: Visual 20cm × 20cm grid showing robot location
- **Real-time Updates**: Robot position and orientation displayed live

## Features

### Visual Display
- ✅ **Blue circle** = Robot position from camera
- ✅ **Yellow arrow** = Robot orientation (yaw angle)
- ✅ **Red crosshair** = Target position (when you click)
- ✅ **Green dashed line** = Path from robot to target
- ✅ **Grid** = 2cm spacing, ±10cm range

### Real-time Information
- **X, Y Position**: Updates live from motion capture (in cm)
- **Yaw Angle**: Robot orientation in degrees
- **Distance to Target**: Automatic calculation
- **Tracking Status**: Shows if robot tag is detected

### Controls
- **Click on grid**: Set target position
- **Reset Origin**: Makes current position the new (0, 0)
- **Clear Target**: Removes target marker

## How to Use

### 1. Start the System
```bash
python pc_controller/integrated_position_control.py
```

### 2. Position Robot
- Place robot with AprilTag visible to camera
- First detection automatically sets origin at (0, 0)
- Blue circle appears on grid showing robot location

### 3. Set Target
- Click anywhere on the grid
- Red crosshair appears at target
- Green line shows path from robot to target
- Distance displays in control panel

### 4. Move Robot
- Move robot by hand (for testing)
- Watch position update in real-time
- See orientation arrow rotate as you turn robot
- Distance to target updates automatically

### 5. Reset if Needed
- Click "Reset Origin" to set new starting position
- Useful if you want to reposition the robot

## Display Information

### Control Panel Shows:
```
Tracking Status: ✓ Tracking Active

Current Position:
  X: +5.23 cm
  Y: -3.45 cm
  Yaw: +12.5°

Target Position: (8.0, 4.0) cm

Distance to Target: 5.87 cm
```

### Canvas Shows:
- 20cm × 20cm workspace (±10cm from origin)
- Grid lines every 2cm
- Origin marked at center (0, 0)
- Robot with heading indicator
- Target crosshair (if set)

## Coordinate System

```
        +Y (up)
         ↑
         |
    -X ←─┼─→ +X
         |
         ↓
        -Y (down)
```

- **Origin (0, 0)** = Where robot was first detected
- **Positive X** = Right
- **Positive Y** = Up
- **Yaw 0°** = Pointing right (+X direction)
- **Yaw 90°** = Pointing up (+Y direction)

## Next Steps

This system provides the foundation for autonomous navigation:

1. **Current**: Manual movement with live position feedback
2. **Next Phase**: Add motor control to drive robot to target
3. **Final Phase**: Closed-loop navigation with drift correction

## Troubleshooting

### Robot not appearing?
- Check AprilTag is visible to camera
- Verify TAG_ID matches (default: 0)
- Check lighting conditions

### Position jumpy?
- Improve lighting (avoid shadows)
- Stabilize camera mount
- Increase smoothing in tracker

### Wrong orientation?
- Yaw shows which way robot is facing
- Arrow should point in robot's forward direction
- If backward, check AprilTag mounting

## Technical Details

- **Update Rate**: ~30 Hz (30 times per second)
- **Position Resolution**: 0.01 cm
- **Angle Resolution**: 0.1 degrees
- **Workspace Size**: 20cm × 20cm (400 cm²)
- **Tracking Method**: AprilTag with pose estimation
