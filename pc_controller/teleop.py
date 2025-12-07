"""
teleop.py

Keyboard teleoperation script for Pololu 3pi+ 2040 robot.

Controls:
    W - Forward
    S - Backward
    A - Turn Left
    D - Turn Right
    SPACE - Stop
    ESC - Quit

Requires: pynput for keyboard input
"""

import sys
import time
import threading
from pynput import keyboard
from robot_controller import RobotController


class RobotTeleop:
    """
    Teleoperation class using keyboard input.
    """
    
    def __init__(self, robot: RobotController):
        """
        Initialize teleoperation.
        
        Args:
            robot: RobotController instance
        """
        self.robot = robot
        self.running = False
        self.current_command = None
        self.last_command_time = 0
        self.command_interval = 0.1  # Send commands every 100ms
        
        # Telemetry display
        self.last_telemetry = ""
        self.telemetry_lock = threading.Lock()
        
    def on_press(self, key):
        """Handle key press events."""
        try:
            # Check for character keys
            if hasattr(key, 'char'):
                if key.char == 'w' or key.char == 'W':
                    self.current_command = 'forward'
                elif key.char == 's' or key.char == 'S':
                    self.current_command = 'backward'
                elif key.char == 'a' or key.char == 'A':
                    self.current_command = 'left'
                elif key.char == 'd' or key.char == 'D':
                    self.current_command = 'right'
            
            # Check for special keys
            if key == keyboard.Key.space:
                self.current_command = 'stop'
            elif key == keyboard.Key.esc:
                print("\nESC pressed - stopping robot and exiting...")
                self.robot.stop()
                self.running = False
                return False  # Stop listener
                
        except AttributeError:
            pass
    
    def on_release(self, key):
        """Handle key release events."""
        # When key is released, stop robot
        self.current_command = 'stop'
    
    def send_commands(self):
        """Continuously send commands to robot."""
        while self.running:
            current_time = time.time()
            
            # Send command at specified interval
            if current_time - self.last_command_time >= self.command_interval:
                if self.current_command == 'forward':
                    self.robot.forward()
                elif self.current_command == 'backward':
                    self.robot.backward()
                elif self.current_command == 'left':
                    self.robot.turn_left()
                elif self.current_command == 'right':
                    self.robot.turn_right()
                elif self.current_command == 'stop':
                    self.robot.stop()
                
                self.last_command_time = current_time
            
            time.sleep(0.01)  # Small sleep to prevent CPU spinning
    
    def read_telemetry(self):
        """Continuously read and display telemetry."""
        while self.running:
            telemetry = self.robot.read_telemetry(timeout=0.1)
            if telemetry:
                with self.telemetry_lock:
                    self.last_telemetry = telemetry
                    # Print telemetry on same line
                    print(f"\r[Telemetry] {telemetry}                    ", end='', flush=True)
            
            time.sleep(0.1)
    
    def run(self):
        """Start teleoperation."""
        print("\n" + "="*60)
        print("ROBOT TELEOPERATION")
        print("="*60)
        print("\nControls:")
        print("  W         - Move Forward")
        print("  S         - Move Backward")
        print("  A         - Turn Left")
        print("  D         - Turn Right")
        print("  SPACE     - Stop")
        print("  ESC       - Quit")
        print("\nStarting in 2 seconds...")
        print("="*60 + "\n")
        
        time.sleep(2)
        
        self.running = True
        self.current_command = 'stop'
        
        # Start command sending thread
        command_thread = threading.Thread(target=self.send_commands, daemon=True)
        command_thread.start()
        
        # Start telemetry reading thread
        telemetry_thread = threading.Thread(target=self.read_telemetry, daemon=True)
        telemetry_thread.start()
        
        # Start keyboard listener
        print("Ready! Press keys to control robot...")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()
        
        self.running = False
        command_thread.join(timeout=1)
        telemetry_thread.join(timeout=1)
        
        print("\n\nTeleoperation ended.")


def main():
    """Main entry point."""
    print("Pololu 3pi+ 2040 Robot - Keyboard Teleoperation")
    print("=" * 60 + "\n")
    
    # Parse command line arguments
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using specified port: {port}\n")
    else:
        print("No port specified, will auto-detect...\n")
    
    # List available ports
    RobotController.list_ports()
    
    # Create robot controller
    robot = RobotController(connection_type='serial', port=port, baudrate=9600)
    
    # Connect to robot
    print("Connecting to robot...")
    if not robot.connect():
        print("\nFailed to connect to robot!")
        print("\nTroubleshooting:")
        print("1. Check that HM-10 module is powered and paired")
        print("2. Verify correct COM port")
        print("3. Ensure no other program is using the port")
        print("4. Check that robot firmware is running")
        print("\nUsage: python teleop.py [PORT]")
        print("Example: python teleop.py COM3")
        sys.exit(1)
    
    try:
        # Create and run teleoperation
        teleop = RobotTeleop(robot)
        teleop.run()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Ensure robot is stopped and disconnected
        print("\nCleaning up...")
        robot.stop()
        time.sleep(0.2)
        robot.disconnect()
        print("Goodbye!\n")


if __name__ == '__main__':
    main()
