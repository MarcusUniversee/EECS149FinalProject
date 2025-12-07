"""
position_control_gui.py

Graphical interface for position-based robot control.
Displays a 20cm x 20cm grid where users can click to send the robot to a target position.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import sys
from robot_controller import RobotController


class PositionControlGUI:
    """
    GUI for visual position control of robot.
    """
    
    def __init__(self, root, robot):
        """
        Initialize GUI.
        
        Args:
            root: Tkinter root window
            robot: RobotController instance
        """
        self.root = root
        self.robot = robot
        
        # Position tracking
        self.robot_x = 0.0  # cm
        self.robot_y = 0.0  # cm
        self.robot_theta = 0.0  # radians
        self.target_x = None
        self.target_y = None
        
        # Canvas settings (20cm x 20cm range, scaled to pixels)
        self.canvas_size = 600  # pixels
        self.world_size = 20.0  # cm (±10cm)
        self.scale = self.canvas_size / self.world_size  # pixels per cm
        
        # Setup GUI
        self.setup_ui()
        
        # Start telemetry updates
        self.update_telemetry()
        
    def setup_ui(self):
        """Create GUI elements."""
        self.root.title("Robot Position Control")
        self.root.geometry("800x700")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, text="Robot Position Controller", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Instructions
        instructions = ttk.Label(main_frame, 
                                text="Click anywhere on the grid to send robot to that position",
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
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        control_frame.grid(row=2, column=1, padx=10, pady=10, sticky=(tk.N, tk.W, tk.E))
        
        # Position display
        ttk.Label(control_frame, text="Current Position:", 
                 font=('Arial', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)
        
        ttk.Label(control_frame, text="X:").grid(row=1, column=0, sticky=tk.E)
        self.x_label = ttk.Label(control_frame, text="0.00 cm", 
                                font=('Arial', 10))
        self.x_label.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Y:").grid(row=2, column=0, sticky=tk.E)
        self.y_label = ttk.Label(control_frame, text="0.00 cm", 
                                font=('Arial', 10))
        self.y_label.grid(row=2, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(control_frame, text="θ:").grid(row=3, column=0, sticky=tk.E)
        self.theta_label = ttk.Label(control_frame, text="0.00°", 
                                     font=('Arial', 10))
        self.theta_label.grid(row=3, column=1, sticky=tk.W, padx=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Target display
        ttk.Label(control_frame, text="Target Position:", 
                 font=('Arial', 10, 'bold')).grid(row=5, column=0, columnspan=2, pady=5)
        
        self.target_label = ttk.Label(control_frame, text="None", 
                                      font=('Arial', 10))
        self.target_label.grid(row=6, column=0, columnspan=2, pady=5)
        
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Buttons
        ttk.Button(control_frame, text="Reset Position", 
                  command=self.reset_position).grid(row=8, column=0, columnspan=2, 
                                                    pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(control_frame, text="Stop Robot", 
                  command=self.stop_robot).grid(row=9, column=0, columnspan=2, 
                                               pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(control_frame, text="Request Position", 
                  command=self.request_position).grid(row=10, column=0, columnspan=2, 
                                                     pady=5, sticky=(tk.W, tk.E))
        
        # Status bar
        self.status_label = ttk.Label(main_frame, text="Ready", 
                                     relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.grid(row=3, column=0, columnspan=2, 
                              sticky=(tk.W, tk.E), pady=5)
        
        # Draw initial grid
        self.draw_grid()
        self.draw_robot()
        
    def draw_grid(self):
        """Draw coordinate grid."""
        # Clear canvas
        self.canvas.delete("grid")
        
        # Grid lines every 2cm
        for i in range(-10, 11, 2):
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
        for i in range(-10, 11, 5):
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
        return int((world_x + 10) * self.scale)
    
    def world_to_canvas_y(self, world_y):
        """Convert world Y coordinate to canvas Y (inverted)."""
        return int((10 - world_y) * self.scale)
    
    def canvas_to_world_x(self, canvas_x):
        """Convert canvas X to world X coordinate."""
        return (canvas_x / self.scale) - 10
    
    def canvas_to_world_y(self, canvas_y):
        """Convert canvas Y to world Y coordinate (inverted)."""
        return 10 - (canvas_y / self.scale)
    
    def draw_robot(self):
        """Draw robot at current position."""
        self.canvas.delete("robot")
        
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
        arrow_length = radius + 10
        end_x = cx + arrow_length * math.cos(self.robot_theta)
        end_y = cy - arrow_length * math.sin(self.robot_theta)  # Negative for canvas coords
        
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
    
    def on_canvas_click(self, event):
        """Handle canvas click to set target."""
        # Convert click position to world coordinates
        world_x = self.canvas_to_world_x(event.x)
        world_y = self.canvas_to_world_y(event.y)
        
        # Round to 1 decimal place
        world_x = round(world_x, 1)
        world_y = round(world_y, 1)
        
        # Check bounds
        if world_x < -10 or world_x > 10 or world_y < -10 or world_y > 10:
            messagebox.showwarning("Out of Bounds", 
                                  "Target position must be within ±10cm range")
            return
        
        # Set target
        self.target_x = world_x
        self.target_y = world_y
        
        # Update display
        self.target_label.config(text=f"({self.target_x:.1f}, {self.target_y:.1f}) cm")
        self.draw_target()
        
        # Send command to robot
        self.send_goto_command(world_x, world_y)
    
    def send_goto_command(self, x, y):
        """Send goto command to robot."""
        try:
            self.status_label.config(text=f"Sending robot to ({x:.1f}, {y:.1f})...")
            self.root.update()
            
            # Send 'g' command followed by coordinates
            command = f"g{x:.2f},{y:.2f}\n"
            self.robot.serial_conn.write(command.encode('utf-8'))
            
            self.status_label.config(text=f"Command sent: Go to ({x:.1f}, {y:.1f})")
            
        except Exception as e:
            messagebox.showerror("Communication Error", f"Failed to send command: {e}")
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
            
            self.x_label.config(text="0.00 cm")
            self.y_label.config(text="0.00 cm")
            self.theta_label.config(text="0.00°")
            self.target_label.config(text="None")
            
            self.draw_robot()
            self.canvas.delete("target")
            
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
    
    def request_position(self):
        """Request current position from robot."""
        try:
            self.robot.send_command(b'p')
            self.status_label.config(text="Position requested...")
        except Exception as e:
            messagebox.showerror("Communication Error", f"Failed to request: {e}")
    
    def update_telemetry(self):
        """Periodically read telemetry from robot."""
        try:
            # Read telemetry
            telemetry = self.robot.read_telemetry(timeout=0.05)
            
            if telemetry:
                # Parse position telemetry
                if telemetry.startswith("POS:"):
                    self.parse_position(telemetry)
                elif telemetry.startswith("NAV:"):
                    self.status_label.config(text=telemetry)
                
        except Exception as e:
            print(f"Telemetry error: {e}")
        
        # Schedule next update
        self.root.after(100, self.update_telemetry)  # Update every 100ms
    
    def parse_position(self, telemetry):
        """Parse position telemetry string."""
        try:
            # Format: "POS: x=1.23 y=4.56 theta=0.78"
            parts = telemetry.split()
            
            for part in parts:
                if part.startswith("x="):
                    self.robot_x = float(part[2:])
                elif part.startswith("y="):
                    self.robot_y = float(part[2:])
                elif part.startswith("theta="):
                    self.robot_theta = float(part[6:])
            
            # Update display
            self.x_label.config(text=f"{self.robot_x:.2f} cm")
            self.y_label.config(text=f"{self.robot_y:.2f} cm")
            theta_deg = math.degrees(self.robot_theta)
            self.theta_label.config(text=f"{theta_deg:.1f}°")
            
            # Redraw robot
            self.draw_robot()
            
        except Exception as e:
            print(f"Error parsing position: {e}")


def main():
    """Main entry point."""
    print("Position Control GUI for Pololu 3pi+ 2040 Robot")
    print("=" * 60 + "\n")
    
    # Parse command line arguments
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using specified port: {port}\n")
    else:
        print("No port specified, will auto-detect...\n")
    
    # Create robot controller
    robot = RobotController(connection_type='serial', port=port, baudrate=9600)
    
    # Connect to robot
    print("Connecting to robot...")
    if not robot.connect():
        print("\nFailed to connect to robot!")
        print("\nTroubleshooting:")
        print("1. Check that HM-10 module is powered and paired")
        print("2. Verify correct COM port")
        print("3. Ensure robot firmware is running")
        print("\nUsage: python position_control_gui.py [PORT]")
        print("Example: python position_control_gui.py COM3")
        sys.exit(1)
    
    print("Connected successfully!\n")
    
    # Create GUI
    root = tk.Tk()
    app = PositionControlGUI(root, robot)
    
    # Handle window close
    def on_closing():
        if messagebox.askokcancel("Quit", "Stop robot and quit?"):
            robot.stop()
            robot.disconnect()
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Run GUI
    print("GUI launched. Click on grid to send robot to target position.\n")
    root.mainloop()
    
    # Cleanup
    robot.disconnect()
    print("\nGoodbye!")


if __name__ == '__main__':
    main()
