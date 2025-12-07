# PC Controller - Python Teleoperation

Python application for controlling the Pololu 3pi+ 2040 robot via keyboard over Bluetooth.

## Overview

This Python application provides:
- Keyboard teleoperation (WASD controls)
- Serial/COM port communication with HM-10
- Real-time telemetry display
- `RobotController` class for easy integration

## Requirements

- **Python 3.10+**
- **Windows** (tested), Linux, or macOS
- **HM-10 Bluetooth module** paired and connected

## Installation

### 1. Install Python Dependencies

```powershell
cd pc_controller
pip install -r requirements.txt
```

This installs:
- `pyserial` - Serial port communication
- `pynput` - Keyboard input handling
- `bleak` - Bluetooth Low Energy (optional, for BLE mode)

### 2. Pair HM-10 Bluetooth Module

#### Windows:
1. Open **Settings** → **Bluetooth & devices**
2. Click **Add device** → **Bluetooth**
3. Select **HMSoft** (or your HM-10 device name)
4. Enter PIN: `0000` or `1234`
5. After pairing, note the assigned COM port (e.g., `COM3`)

#### Check COM Port:
```powershell
mode  # List all COM ports
# or
Get-WmiObject Win32_SerialPort | Select-Object Name,DeviceID
```

## Usage

### Keyboard Teleoperation

#### Auto-detect COM port:
```powershell
python teleop.py
```

#### Specify COM port:
```powershell
python teleop.py COM3
```

### Position Control GUI (NEW!)

Launch visual interface for position-based navigation:

```powershell
python position_control_gui.py
```

Or specify port:
```powershell
python position_control_gui.py COM3
```

**Features:**
- Click anywhere on 20cm × 20cm grid
- Robot automatically navigates to clicked position
- Real-time position tracking and display
- Visual indicators for robot and target
- Emergency stop and position reset

**See `POSITION_CONTROL_GUIDE.md` for complete documentation.**

### Controls

| Key   | Action          |
|-------|-----------------|
| W     | Move Forward    |
| S     | Move Backward   |
| A     | Turn Left       |
| D     | Turn Right      |
| SPACE | Stop            |
| ESC   | Quit Program    |

**Note**: Keys must be held down for continuous movement. Robot stops when key is released.

## Files

### `robot_controller.py`

Main communication class for robot control.

#### Key Features:
- Serial port auto-detection
- Command sending (`forward()`, `backward()`, etc.)
- Telemetry reading
- Connection management

#### Example Usage:

```python
from robot_controller import RobotController
import time

# Create controller
robot = RobotController(connection_type='serial', port='COM3')

# Connect
if robot.connect():
    # Send commands
    robot.forward()
    time.sleep(2)
    robot.stop()
    
    # Read telemetry
    telemetry = robot.read_telemetry()
    print(telemetry)
    
    # Disconnect
    robot.disconnect()
```

#### API Reference:

```python
class RobotController:
    def __init__(connection_type='serial', port=None, baudrate=9600)
    def connect() -> bool
    def disconnect()
    def send_command(command: bytes) -> bool
    def read_telemetry(timeout=0.1) -> Optional[str]
    
    # Convenience methods
    def forward()
    def backward()
    def turn_left()
    def turn_right()
    def stop()
    
    # Position control (NEW)
    def goto_position(x: float, y: float) -> bool
    def get_position() -> bool
    def reset_position() -> bool
    
    # Utility
    @staticmethod
    def list_ports()
```

### `teleop.py`

Keyboard teleoperation script using `pynput`.

#### Features:
- Non-blocking keyboard input
- Continuous command sending (10 Hz)
- Real-time telemetry display
- Graceful shutdown on ESC

#### Running:

```powershell
python teleop.py [COM_PORT]
```

### `position_control_gui.py` (NEW)

Graphical interface for visual position control.

#### Features:
- 20cm × 20cm clickable grid display
- Real-time robot position visualization
- Click-to-navigate functionality
- Position reset and emergency stop
- Telemetry display

#### Running:

```powershell
python position_control_gui.py [COM_PORT]
```

**Full documentation**: See `POSITION_CONTROL_GUIDE.md`

### `requirements.txt`

Python package dependencies:
- `pyserial` - Serial communication
- `pynput` - Keyboard event handling
- `bleak` - BLE support (future enhancement)

## Testing Connection

### Test Script

The `robot_controller.py` can be run standalone to test connection:

```powershell
python robot_controller.py
```

This will:
1. List available COM ports
2. Auto-connect to robot
3. Execute test command sequence
4. Read telemetry for 5 seconds
5. Disconnect

### Manual Testing with Serial Terminal

Use PuTTY, Tera Term, or similar:

1. Open serial connection to HM-10 COM port
2. Settings: 9600 baud, 8N1, no flow control
3. Type commands: `f`, `b`, `l`, `r`, `s`
4. Observe telemetry output

## Troubleshooting

### Cannot Find COM Port

**Issue**: `No serial port found` or connection fails

**Solutions**:
1. Verify HM-10 is paired in Bluetooth settings
2. Check Device Manager for COM port assignment
3. Manually specify port: `python teleop.py COM3`
4. Try unpair/repair HM-10

### Permission Denied (Linux/macOS)

**Issue**: `Permission denied: '/dev/ttyUSB0'`

**Solution**:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Commands Not Working

**Issue**: Commands sent but robot doesn't respond

**Solutions**:
1. Check robot firmware is running (LED blinking)
2. Verify UART wiring (TX/RX not swapped)
3. Check baud rate matches (9600)
4. Test with manual serial terminal first

### Keyboard Not Responding

**Issue**: Keys pressed but no robot movement

**Solutions**:
1. Ensure teleop script has focus (click terminal window)
2. Check `pynput` installed: `pip install pynput`
3. Try running as administrator (Windows)
4. Linux: May need X11 display

### High Latency

**Issue**: Delayed response to keyboard input

**Solutions**:
1. Reduce command interval in `teleop.py`:
   ```python
## Advanced Usage

### Position Control

Navigate robot to specific coordinates:

```python
from robot_controller import RobotController
import time

robot = RobotController(port='COM3')
robot.connect()

# Navigate to position (5.0, -3.0) cm
robot.goto_position(5.0, -3.0)

# Wait for arrival
time.sleep(5)

# Reset position to origin
robot.reset_position()

robot.disconnect()
```

### Custom Control Script strength (move closer)
3. Reduce telemetry rate on robot firmware

## Advanced Usage

### Custom Control Script

Create your own control script using `RobotController`:

```python
from robot_controller import RobotController
import time

robot = RobotController(port='COM3')
robot.connect()

# Square pattern
for _ in range(4):
    robot.forward()
    time.sleep(1)
    robot.turn_right()
    time.sleep(0.5)

robot.stop()
robot.disconnect()
```

### Telemetry Callback

Register callback for telemetry:

```python
def handle_telemetry(data):
    print(f"Received: {data}")

robot = RobotController(port='COM3')
robot.connect()
robot.set_telemetry_callback(handle_telemetry)
```

### BLE Mode (Future)

```python
# Not yet implemented
robot = RobotController(
    connection_type='ble',
    port='XX:XX:XX:XX:XX:XX'  # HM-10 MAC address
)
```

## Future Enhancements

- [x] GUI control interface (position_control_gui.py)
- [x] Position-based navigation
- [ ] Joystick/gamepad support
- [ ] Telemetry plotting and logging
- [ ] Autonomous mode integration
- [ ] Multi-robot control
- [ ] Video streaming integration
- [ ] BLE UART implementation with `bleak`
- [ ] Path planning with obstacle avoidance
Higher values (0.2) = Less responsive, lower bandwidth
Lower values (0.05) = More responsive, higher bandwidth

### Change Baud Rate

Edit both `robot_controller.py` and robot firmware:
```python
robot = RobotController(baudrate=115200)
```

**Note**: Must also reconfigure HM-10 module baud rate

## Performance

- **Command Latency**: 50-150ms (depends on Bluetooth)
- **Command Rate**: 10 Hz (adjustable)
- **Telemetry Rate**: 1 Hz (robot firmware setting)
- **Range**: ~10 meters line-of-sight

## Future Enhancements

- [ ] GUI control interface (Tkinter or PyQt)
- [ ] Joystick/gamepad support
- [ ] Telemetry plotting and logging
- [ ] Autonomous mode integration
- [ ] Multi-robot control
- [ ] Video streaming integration
- [ ] BLE UART implementation with `bleak`

## Dependencies Details

### pyserial
- Cross-platform serial port library
- Used for COM port communication
- [Documentation](https://pyserial.readthedocs.io/)

### pynput
- Keyboard and mouse input monitoring
- Used for non-blocking key detection
- [Documentation](https://pynput.readthedocs.io/)

### bleak
- Bluetooth Low Energy platform agnostic library
- For future BLE UART support
- [Documentation](https://bleak.readthedocs.io/)

## Common Issues

### "Module not found" Error

```powershell
pip install --upgrade -r requirements.txt
```

### Serial Port Already in Use

Close other programs using the port:
- Arduino IDE Serial Monitor
- PuTTY/Tera Term
- Other Python scripts

### HM-10 Not Pairing

1. Reset HM-10 module (power cycle)
2. Remove pairing and re-pair
3. Check HM-10 LED (should blink when not connected)
4. Try AT commands to reset: `AT+RESET`

## License

MIT License - Free to use and modify.

## Support

For issues:
- Check main repository README troubleshooting section
- Verify robot firmware is running correctly
- Test with manual serial terminal first

---

**Happy Driving! 🎮🤖**
