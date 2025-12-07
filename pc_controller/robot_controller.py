"""
robot_controller.py

RobotController class for communicating with Pololu 3pi+ 2040 robot
via HM-10 Bluetooth module.

Supports both:
- Serial/COM port connection (pyserial)
- BLE UART connection (bleak)
"""

import serial
import serial.tools.list_ports
import time
import sys
from typing import Optional, Callable


class RobotController:
    """
    Controller class for Pololu 3pi+ 2040 robot via HM-10 Bluetooth.
    
    Supports both serial port and BLE connections.
    """
    
    # Command definitions
    CMD_FORWARD = b'f'
    CMD_BACKWARD = b'b'
    CMD_LEFT = b'l'
    CMD_RIGHT = b'r'
    CMD_STOP = b's'
    CMD_GOTO = b'g'       # Go to position
    CMD_GET_POS = b'p'    # Request position
    CMD_RESET_POS = b'z'  # Reset position
    
    def __init__(self, connection_type='serial', port=None, baudrate=9600):
        """
        Initialize robot controller.
        
        Args:
            connection_type: 'serial' or 'ble'
            port: COM port name (e.g., 'COM3') for serial, or device address for BLE
            baudrate: Serial baud rate (default 9600)
        """
        self.connection_type = connection_type
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.ble_conn = None
        self.connected = False
        self.telemetry_callback = None
        
    def connect(self) -> bool:
        """
        Connect to robot via serial or BLE.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        if self.connection_type == 'serial':
            return self._connect_serial()
        elif self.connection_type == 'ble':
            return self._connect_ble()
        else:
            print(f"Error: Unknown connection type '{self.connection_type}'")
            return False
    
    def _connect_serial(self) -> bool:
        """Connect via serial port."""
        try:
            if self.port is None:
                # Auto-detect port
                self.port = self._find_serial_port()
                if self.port is None:
                    print("Error: No serial port found")
                    return False
            
            # ===================================================================
            # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
            # ===================================================================
            # USB connection uses 115200 baud (native USB CDC serial)
            # Bluetooth uses 9600 baud (HM-10 default)
            original_baudrate = self.baudrate
            
            # Check if this looks like a USB connection
            if 'COM' in self.port.upper():
                print(f"[TEMP USB MODE] Trying USB baudrate (115200) first...")
                self.baudrate = 115200  # Pico USB CDC default
            # ===================================================================
            # END TEMPORARY USB-C TESTING CODE
            # ===================================================================
            
            print(f"Connecting to {self.port} at {self.baudrate} baud...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Give it a moment to initialize
            time.sleep(0.5)
            
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # ===================================================================
            # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
            # ===================================================================
            # Test connection by sending stop command
            print("[TEMP USB MODE] Testing connection...")
            self.serial_conn.write(b's')
            time.sleep(0.2)
            
            # If we get here, connection is good
            print(f"[TEMP USB MODE] USB connection successful at {self.baudrate} baud")
            # ===================================================================
            # END TEMPORARY USB-C TESTING CODE
            # ===================================================================
            
            self.connected = True
            print(f"Connected to robot on {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            # ===================================================================
            # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
            # ===================================================================
            print("[TEMP USB MODE] Tip: Make sure USB cable is connected to Pico")
            print("[TEMP USB MODE] Tip: Robot should appear as 'USB Serial Device' or similar")
            # ===================================================================
            # END TEMPORARY USB-C TESTING CODE
            # ===================================================================
            return False
    
    def _connect_ble(self) -> bool:
        """Connect via BLE (placeholder for bleak implementation)."""
        print("BLE connection not yet implemented")
        print("Please use serial connection type")
        # TODO: Implement BLE connection using bleak library
        return False
    
    def _find_serial_port(self) -> Optional[str]:
        """
        Auto-detect robot serial port.
        
        Returns:
            Optional[str]: Port name if found, None otherwise
        """
        ports = serial.tools.list_ports.comports()
        
        # ===================================================================
        # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
        # ===================================================================
        # Look for Raspberry Pi Pico (RP2040) connected via USB-C
        # This bypasses Bluetooth for direct USB serial communication
        usb_keywords = ['pico', 'rp2040', 'raspberry pi', 'usb serial']
        
        print("\n[TEMP USB MODE] Searching for USB-C connected Pico...")
        for port in ports:
            port_info = f"{port.device} {port.description} {port.manufacturer}".lower()
            print(f"  Found: {port.device} - {port.description}")
            
            # Check for USB-connected Pico
            for keyword in usb_keywords:
                if keyword in port_info:
                    print(f"[TEMP USB MODE] Found Pico via USB: {port.device}")
                    return port.device
        # ===================================================================
        # END TEMPORARY USB-C TESTING CODE
        # ===================================================================
        
        # Look for common Bluetooth serial adapter names
        keywords = ['bluetooth', 'hm-10', 'hm10', 'bt', 'serial']
        
        for port in ports:
            port_info = f"{port.device} {port.description} {port.manufacturer}".lower()
            for keyword in keywords:
                if keyword in port_info:
                    print(f"Found potential robot port: {port.device} ({port.description})")
                    return port.device
        
        # If no keyword match, return first available port (risky!)
        if len(ports) > 0:
            print(f"No Bluetooth port detected, using first available: {ports[0].device}")
            return ports[0].device
        
        return None
    
    def disconnect(self):
        """Disconnect from robot."""
        if self.serial_conn and self.serial_conn.is_open:
            # Send stop command before disconnecting
            self.send_command(self.CMD_STOP)
            time.sleep(0.1)
            self.serial_conn.close()
            print("Disconnected from robot")
        
        self.connected = False
    
    def send_command(self, command: bytes) -> bool:
        """
        Send single-byte command to robot.
        
        Args:
            command: Single byte command (f/b/l/r/s)
            
        Returns:
            bool: True if sent successfully, False otherwise
        """
        if not self.connected:
            print("Error: Not connected to robot")
            return False
        
        try:
            if self.serial_conn:
                self.serial_conn.write(command)
                return True
            elif self.ble_conn:
                # TODO: Implement BLE write
                pass
                
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
        
        return False
    
    def read_telemetry(self, timeout: float = 0.1) -> Optional[str]:
        """
        Read telemetry data from robot.
        
        Args:
            timeout: Read timeout in seconds
            
        Returns:
            Optional[str]: Telemetry string if available, None otherwise
        """
        if not self.connected or not self.serial_conn:
            return None
        
        try:
            # Set temporary timeout
            old_timeout = self.serial_conn.timeout
            self.serial_conn.timeout = timeout
            
            # Read line (terminated with \n)
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline()
                self.serial_conn.timeout = old_timeout
                
                try:
                    return line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    return None
            
            self.serial_conn.timeout = old_timeout
            
        except Exception as e:
            print(f"Error reading telemetry: {e}")
        
        return None
    
    def set_telemetry_callback(self, callback: Callable[[str], None]):
        """
        Set callback function for telemetry data.
        
        Args:
            callback: Function that takes telemetry string as argument
        """
        self.telemetry_callback = callback
    
    def forward(self):
        """Move forward."""
        self.send_command(self.CMD_FORWARD)
    
    def backward(self):
        """Move backward."""
        self.send_command(self.CMD_BACKWARD)
    
    def turn_left(self):
        """Turn left."""
        self.send_command(self.CMD_LEFT)
    
    def turn_right(self):
        """Turn right."""
        self.send_command(self.CMD_RIGHT)
    
    def stop(self):
        """Stop motors."""
        self.send_command(self.CMD_STOP)
    
    def goto_position(self, x: float, y: float) -> bool:
        """
        Send robot to target position.
        
        Args:
            x: Target X coordinate in cm
            y: Target Y coordinate in cm
            
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            print("Error: Not connected to robot")
            return False
        
        try:
            # Send 'g' command followed by "x,y\n"
            command = f"g{x:.2f},{y:.2f}\n"
            if self.serial_conn:
                self.serial_conn.write(command.encode('utf-8'))
                print(f"Sent goto command: ({x:.2f}, {y:.2f})")
                return True
                
        except Exception as e:
            print(f"Error sending goto command: {e}")
            return False
        
        return False
    
    def get_position(self) -> bool:
        """
        Request current position from robot.
        
        Returns:
            bool: True if request sent successfully
        """
        return self.send_command(self.CMD_GET_POS)
    
    def reset_position(self) -> bool:
        """
        Reset robot position to origin (0, 0).
        
        Returns:
            bool: True if command sent successfully
        """
        return self.send_command(self.CMD_RESET_POS)
    
    @staticmethod
    def list_ports():
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        print("\nAvailable serial ports:")
        if len(ports) == 0:
            print("  No ports found")
        else:
            # ===================================================================
            # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
            # ===================================================================
            print("  [TEMP USB MODE] Looking for Pico or Bluetooth devices...")
            # ===================================================================
            # END TEMPORARY USB-C TESTING CODE
            # ===================================================================
            for port in ports:
                print(f"  {port.device}: {port.description}")
                # ===================================================================
                # TEMPORARY USB-C TESTING CODE - DELETE WHEN BLUETOOTH IS WORKING
                # ===================================================================
                # Highlight USB devices
                port_lower = f"{port.description} {port.manufacturer}".lower()
                if 'pico' in port_lower or 'rp2040' in port_lower or 'usb serial' in port_lower:
                    print(f"    ^^^ [TEMP USB MODE] This looks like USB-connected Pico!")
                # ===================================================================
                # END TEMPORARY USB-C TESTING CODE
                # ===================================================================
        print()


def test_connection(port=None):
    """
    Test robot connection and basic commands.
    
    Args:
        port: Serial port name (None for auto-detect)
    """
    print("=== Robot Connection Test ===\n")
    
    # List available ports
    RobotController.list_ports()
    
    # Create controller
    robot = RobotController(connection_type='serial', port=port)
    
    # Connect
    if not robot.connect():
        print("Failed to connect to robot")
        return
    
    try:
        # Send test commands
        print("\nTesting commands...")
        
        print("Forward...")
        robot.forward()
        time.sleep(1)
        
        print("Stop...")
        robot.stop()
        time.sleep(1)
        
        print("Backward...")
        robot.backward()
        time.sleep(1)
        
        print("Stop...")
        robot.stop()
        time.sleep(1)
        
        print("Left...")
        robot.turn_left()
        time.sleep(1)
        
        print("Stop...")
        robot.stop()
        time.sleep(1)
        
        print("Right...")
        robot.turn_right()
        time.sleep(1)
        
        print("Stop...")
        robot.stop()
        
        # Read telemetry
        print("\nReading telemetry for 5 seconds...")
        start_time = time.time()
        while time.time() - start_time < 5:
            telemetry = robot.read_telemetry()
            if telemetry:
                print(f"  {telemetry}")
            time.sleep(0.1)
        
        print("\nTest complete!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    # Run connection test
    test_connection()
