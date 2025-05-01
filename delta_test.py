import math
import random
import time
import serial
from typing import Tuple, List

class DeltaRobot:
    def __init__(self, 
                 upper_arm_length: float = 150.0, 
                 lower_arm_length: float = 550.0,
                 upper_base_radius: float = 190.0,
                 effector_radius: float = 70.0,
                 initial_angle: float = 45.0 * (math.pi / 180.0),  # 45 degrees in radians
                 motor_min_angle: float = -30.0 * (math.pi / 180.0),  # Minimum motor angle in radians
                 motor_max_angle: float = 30.0 * (math.pi / 180.0)):  # Maximum motor angle in radians
        
        # Delta robot parameters
        self.upper_arm_length = upper_arm_length  # Length of the upper arm (mm)
        self.lower_arm_length = lower_arm_length  # Length of the lower arm (mm)
        self.upper_base_radius = upper_base_radius  # Radius of the fixed base (mm)
        self.effector_radius = effector_radius  # Radius of the end effector (mm)
        self.initial_angle = initial_angle  # Initial angle between upper arm and horizontal (radians)
        
        # Motor angle limits (radians)
        self.motor_min_angle = motor_min_angle
        self.motor_max_angle = motor_max_angle
        
        # Calculate the position of motor joints in base reference frame
        self.base_joints = []
        for i in range(3):
            angle = 2 * math.pi * i / 3
            x = self.upper_base_radius * math.cos(angle)
            y = self.upper_base_radius * math.sin(angle)
            z = 0
            self.base_joints.append((x, y, z))
        
        # Calculate the position of end effector joints in end effector reference frame
        self.effector_joints = []
        for i in range(3):
            angle = 2 * math.pi * i / 3
            x = self.effector_radius * math.cos(angle)
            y = self.effector_radius * math.sin(angle)
            z = 0
            self.effector_joints.append((x, y, z))
        
        # Steps per revolution for the stepper motors
        self.steps_per_rev = 200  # Typical for many stepper motors
        
        # Microstepping setting
        self.microsteps = 16  # Common microstepping setting
        
        # Total steps per revolution with microstepping
        self.total_steps_per_rev = self.steps_per_rev * self.microsteps
        
        # Calculate work envelope
        self.calculate_work_envelope()
        
        # Initialize serial connection for CNC shield
        self.serial_connection = None
    
    def calculate_work_envelope(self):
        """Calculate the approximate work envelope of the delta robot."""
        # Calculate the lowest point the end effector can reach
        # This is a simplified calculation, actually finding the exact envelope is complex
        
        # Maximum extension height (when arms are straight down)
        self.z_min = -math.sqrt(
            (self.upper_arm_length + self.lower_arm_length)**2 - 
            (self.upper_base_radius - self.effector_radius)**2
        )
        
        # Maximum height - simplified approximation
        self.z_max = -self.upper_arm_length * math.sin(self.initial_angle) + self.lower_arm_length * 0.5
        
        # XY work radius varies with height, but at z=0 it's approximately:
        self.xy_radius_at_z0 = math.sqrt(
            (self.lower_arm_length)**2 - 
            (self.upper_arm_length * math.sin(self.initial_angle))**2
        ) - (self.upper_base_radius - self.effector_radius)
        
        print(f"Approximate work envelope:")
        print(f"Z range: {self.z_min:.2f}mm to {self.z_max:.2f}mm")
        print(f"XY radius at z=0: approximately {self.xy_radius_at_z0:.2f}mm")
    
    def connect_serial(self, port='/dev/ttyACM0', baud_rate=115200):
        """Connect to the CNC shield through serial."""
        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            print(f"Connected to {port} at {baud_rate} baud")
            # Send initial configuration to GRBL controller
            self.send_gcode("$X")  # Unlock
            time.sleep(0.5)
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False
    
    def send_gcode(self, command):
        """Send G-code command to the CNC shield."""
        if self.serial_connection and self.serial_connection.is_open:
            command_with_newline = f"{command}\n"
            self.serial_connection.write(command_with_newline.encode())
            # Wait for response/acknowledgment
            response = self.serial_connection.readline().decode().strip()
            print(f"Sent: {command} | Response: {response}")
            return response
        else:
            print(f"Serial not connected. Command not sent: {command}")
            return None
    
    def inverse_kinematics(self, x: float, y: float, z: float) -> List[float]:
        """
        Calculate motor angles required to position end effector at (x,y,z).
        Returns a list of the three motor angles in radians.
        """
        motor_angles = []
        
        for i in range(3):
            # Position of the i-th base joint
            base_x, base_y, base_z = self.base_joints[i]
            
            # Position of the i-th effector joint in global coordinates
            effector_x = x + self.effector_joints[i][0]
            effector_y = y + self.effector_joints[i][1]
            effector_z = z + self.effector_joints[i][2]
            
            # Calculate the horizontal distance from base joint to end effector joint
            dx = effector_x - base_x
            dy = effector_y - base_y
            horizontal_dist = math.sqrt(dx*dx + dy*dy)
            
            # Calculate vertical distance
            dz = effector_z - base_z
            
            # Calculate elbow-to-effector horizontal distance
            horizontal_offset = horizontal_dist - self.upper_base_radius + self.effector_radius
            
            # Using law of cosines to calculate the upper arm angle
            # First, calculate the distance from motor joint to effector joint
            F = math.sqrt(horizontal_offset*horizontal_offset + dz*dz)
            
            # Using law of cosines to find angle
            cos_theta = (self.upper_arm_length*self.upper_arm_length + F*F - self.lower_arm_length*self.lower_arm_length) / (2 * self.upper_arm_length * F)
            
            # Clamp to avoid domain errors
            cos_theta = max(-1.0, min(1.0, cos_theta))
            
            # Calculate the angle between upper arm and horizontal line
            theta1 = math.acos(cos_theta)
            
            # Calculate the angle between F and horizontal line
            alpha = math.atan2(dz, horizontal_offset)
            
            # Final motor angle (deviation from initial position)
            motor_angle = alpha - theta1 - self.initial_angle
            
            # Check if angle is within limits
            if motor_angle < self.motor_min_angle or motor_angle > self.motor_max_angle:
                print(f"Warning: Motor {i+1} angle {math.degrees(motor_angle):.2f}° is outside limits "
                      f"[{math.degrees(self.motor_min_angle):.2f}°, {math.degrees(self.motor_max_angle):.2f}°]")
            
            motor_angles.append(motor_angle)
        
        return motor_angles
    
    def angles_to_steps(self, angles):
        """Convert motor angles to stepper motor steps."""
        steps = []
        for angle in angles:
            # Convert angle to steps
            # Full circle is 2*pi radians which corresponds to steps_per_rev steps
            steps_float = angle * self.total_steps_per_rev / (2 * math.pi)
            steps.append(int(round(steps_float)))
        return steps
    
    def calculate_motor_commands(self, x: float, y: float, z: float) -> str:
        """Calculate motor positions and return as G-code string."""
        # Get the angles for each motor
        motor_angles = self.inverse_kinematics(x, y, z)
        
        # Convert angles to steps
        steps = self.angles_to_steps(motor_angles)
        
        # Format as G-code
        return f"G1 X{steps[0]} Y{steps[1]} Z{steps[2]} F1000"
    
    def move_to_position(self, x: float, y: float, z: float, feed_rate: float = 1000):
        """Move to a specific XYZ position."""
        print(f"Moving to position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        
        # Calculate motor commands
        gcode = self.calculate_motor_commands(x, y, z)
        
        # Add feed rate
        if "F" not in gcode:
            gcode += f" F{feed_rate}"
        
        # Send the command
        self.send_gcode(gcode)
    
    def move_in_circle(self, radius: float, z_height: float, steps: int = 50, revolutions: float = 1.0, delay: float = 0.05):
        """
        Move the end effector in a horizontal circular path.
        
        Args:
            radius (float): Radius of the circle in mm
            z_height (float): Fixed Z height for the circular movement
            steps (int): Number of steps to complete one revolution
            revolutions (float): Number of complete revolutions to perform
            delay (float): Delay between steps in seconds
        """
        if not self.serial_connection:
            print("Serial connection not established. Connect first.")
            return
            
        # Safety check - make sure the requested circle is within the work envelope
        if radius > self.xy_radius_at_z0 or z_height < self.z_min or z_height > self.z_max:
            print(f"WARNING: Requested circle (radius={radius}, z={z_height}) may be outside work envelope.")
            print(f"Maximum recommended radius at z=0: {self.xy_radius_at_z0:.2f}mm")
            print(f"Z range: {self.z_min:.2f}mm to {self.z_max:.2f}mm")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        print(f"Moving in a circle of radius {radius}mm at height {z_height}mm")
        print(f"Performing {revolutions} revolutions with {steps} steps each")
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        # Initial move to the starting position of the circle
        start_x = radius  # Start at (radius, 0)
        start_y = 0
        self.move_to_position(start_x, start_y, z_height)
        time.sleep(1)  # Give time to reach the starting position
        
        # Calculate total number of steps
        total_steps = int(steps * revolutions)
        
        # Generate and execute the circular path
        for i in range(total_steps + 1):  # +1 to complete the circle
            # Calculate the angle for this step
            angle = 2 * math.pi * i / steps
            
            # Calculate position on the circle
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # Move to the position
            self.move_to_position(x, y, z_height)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (total_steps // 10) == 0 or i == total_steps:
                progress = (i / total_steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("Circular movement completed")
    
    def move_in_spiral(self, start_radius: float, end_radius: float, z_height: float, 
                     steps: int = 200, revolutions: float = 2.0, delay: float = 0.05):
        """
        Move the end effector in a horizontal spiral path (from inner to outer or vice versa).
        
        Args:
            start_radius (float): Starting radius of the spiral in mm
            end_radius (float): Ending radius of the spiral in mm
            z_height (float): Fixed Z height for the spiral movement
            steps (int): Number of steps to complete the spiral
            revolutions (float): Number of complete revolutions to perform
            delay (float): Delay between steps in seconds
        """
        if not self.serial_connection:
            print("Serial connection not established. Connect first.")
            return
            
        # Safety check - make sure the requested spiral is within the work envelope
        max_radius = max(abs(start_radius), abs(end_radius))
        if max_radius > self.xy_radius_at_z0 or z_height < self.z_min or z_height > self.z_max:
            print(f"WARNING: Requested spiral (max radius={max_radius}, z={z_height}) may be outside work envelope.")
            print(f"Maximum recommended radius at z=0: {self.xy_radius_at_z0:.2f}mm")
            print(f"Z range: {self.z_min:.2f}mm to {self.z_max:.2f}mm")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        print(f"Moving in a spiral from radius {start_radius}mm to {end_radius}mm at height {z_height}mm")
        print(f"Performing {revolutions} revolutions with {steps} steps")
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        # Initial move to the starting position of the spiral
        start_x = start_radius  # Start at (start_radius, 0)
        start_y = 0
        self.move_to_position(start_x, start_y, z_height)
        time.sleep(1)  # Give time to reach the starting position
        
        # Generate and execute the spiral path
        for i in range(steps + 1):  # +1 to complete the path
            # Calculate the angle and radius for this step
            angle = 2 * math.pi * revolutions * i / steps
            radius = start_radius + (end_radius - start_radius) * i / steps
            
            # Calculate position on the spiral
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # Move to the position
            self.move_to_position(x, y, z_height)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (steps // 10) == 0 or i == steps:
                progress = (i / steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("Spiral movement completed")

    def move_in_figure_eight(self, width: float, height: float, z_height: float, 
                          steps: int = 200, cycles: float = 3.0, delay: float = 0.05):
        """
        Move the end effector in a horizontal figure-eight pattern.
        
        Args:
            width (float): Width of the figure-eight in mm
            height (float): Height of the figure-eight in mm
            z_height (float): Fixed Z height for the movement
            steps (int): Number of steps to complete one cycle
            cycles (float): Number of complete cycles to perform
            delay (float): Delay between steps in seconds
        """
        if not self.serial_connection:
            print("Serial connection not established. Connect first.")
            return
            
        # Safety check - make sure the requested pattern is within the work envelope
        max_radius = math.sqrt((width/2)**2 + (height/2)**2)
        if max_radius > self.xy_radius_at_z0 or z_height < self.z_min or z_height > self.z_max:
            print(f"WARNING: Requested figure-eight (max radius={max_radius}, z={z_height}) may be outside work envelope.")
            print(f"Maximum recommended radius at z=0: {self.xy_radius_at_z0:.2f}mm")
            print(f"Z range: {self.z_min:.2f}mm to {self.z_max:.2f}mm")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        print(f"Moving in a figure-eight with width {width}mm and height {height}mm at z={z_height}mm")
        print(f"Performing {cycles} cycles with {steps} steps each")
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        # Calculate total number of steps
        total_steps = int(steps * cycles)
        
        # Generate and execute the figure-eight path
        for i in range(total_steps + 1):  # +1 to complete the path
            # Use parametric equation for a figure-eight (lemniscate of Gerono)
            t = 2 * math.pi * i / steps
            
            # Calculate position on the figure-eight
            x = (width/2) * math.sin(t)
            y = (height/2) * math.sin(t) * math.cos(t)
            
            # Move to the position
            self.move_to_position(x, y, z_height)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (total_steps // 10) == 0 or i == total_steps:
                progress = (i / total_steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("Figure-eight movement completed")
    
    def move_in_3d_spiral(self, radius: float, start_z: float, end_z: float, 
                         steps: int = 200, revolutions: float = 3.0, delay: float = 0.05):
        """
        Move the end effector in a 3D spiral path, changing height as it rotates.
        
        Args:
            radius (float): Radius of the spiral in mm
            start_z (float): Starting Z height for the spiral movement
            end_z (float): Ending Z height for the spiral movement
            steps (int): Number of steps to complete the spiral
            revolutions (float): Number of complete revolutions to perform
            delay (float): Delay between steps in seconds
        """
        if not self.serial_connection:
            print("Serial connection not established. Connect first.")
            return
            
        # Safety check - make sure the requested spiral is within the work envelope
        if radius > self.xy_radius_at_z0 or start_z < self.z_min or start_z > self.z_max or end_z < self.z_min or end_z > self.z_max:
            print(f"WARNING: Requested 3D spiral may be outside work envelope.")
            print(f"Maximum recommended radius at z=0: {self.xy_radius_at_z0:.2f}mm")
            print(f"Z range: {self.z_min:.2f}mm to {self.z_max:.2f}mm")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        print(f"Moving in a 3D spiral with radius {radius}mm from z={start_z}mm to z={end_z}mm")
        print(f"Performing {revolutions} revolutions with {steps} steps")
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        # Initial move to the starting position of the spiral
        start_x = radius  # Start at (radius, 0)
        start_y = 0
        self.move_to_position(start_x, start_y, start_z)
        time.sleep(1)  # Give time to reach the starting position
        
        # Calculate total number of steps
        total_steps = int(steps * revolutions)
        
        # Generate and execute the 3D spiral path
        for i in range(total_steps + 1):  # +1 to complete the path
            # Calculate the angle for this step
            angle = 2 * math.pi * i / steps
            
            # Calculate position on the spiral
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # Calculate z position - linear interpolation from start_z to end_z
            z = start_z + (end_z - start_z) * (i / total_steps)
            
            # Move to the position
            self.move_to_position(x, y, z)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (total_steps // 10) == 0 or i == total_steps:
                progress = (i / total_steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("3D spiral movement completed")
    
    def close(self):
        """Close the serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed")


# Main execution
if __name__ == "__main__":
    # Create delta robot instance with your specified parameters
    delta = DeltaRobot(
        upper_arm_length=150.0,  # 150mm upper limb
        lower_arm_length=550.0,  # 550mm lower limb
        upper_base_radius=190.0,  # 190mm upper support radius 
        effector_radius=70.0,     # 70mm end effector radius
        initial_angle=math.pi/4,  # 45 degrees initial angle
        motor_min_angle=-30.0 * (math.pi / 180.0),  # -30 degrees
        motor_max_angle=30.0 * (math.pi / 180.0)    # +30 degrees
    )
    
    # Connect to the CNC shield via serial
    port = input("Enter serial port (default: /dev/ttyACM0): ") or "/dev/ttyACM0"
    
    if delta.connect_serial(port=port):
        try:
            print("Choose a movement pattern:")
            print("1. Circle")
            print("2. Spiral")
            print("3. Figure-eight")
            print("4. 3D spiral")
            choice = input("Enter your choice (1-4): ")
            
            # Default Z height in the middle of the work envelope
            z_height = (delta.z_min + delta.z_max) / 2
            
            if choice == "1":
                # Circle parameters
                circle_radius = float(input("Enter circle radius in mm (recommended: 100): ") or "100")
                z_height = float(input(f"Enter Z height in mm (recommended: {z_height:.1f}): ") or f"{z_height}")
                steps_per_rev = int(input("Enter steps per revolution (recommended: 60): ") or "60")
                num_revolutions = float(input("Enter number of revolutions (recommended: 3): ") or "3")
                step_delay = float(input("Enter delay between steps in seconds (recommended: 0.1): ") or "0.1")
                
                print("\nStarting circular movement...")
                # Execute the circular movement
                delta.move_in_circle(
                    radius=circle_radius,
                    z_height=z_height,
                    steps=steps_per_rev,
                    revolutions=num_revolutions,
                    delay=step_delay
                )
                
            elif choice == "2":
                # Spiral parameters
                start_radius = float(input("Enter starting radius in mm (recommended: 20): ") or "20")
                end_radius = float(input("Enter ending radius in mm (recommended: 150): ") or "150")
                z_height = float(input(f"Enter Z height in mm (recommended: {z_height:.1f}): ") or f"{z_height}")
                steps = int(input("Enter total steps (recommended: 200): ") or "200")
                revolutions = float(input("Enter number of revolutions (recommended: 2): ") or "2")
                delay = float(input("Enter delay between steps in seconds (recommended: 0.05): ") or "0.05")
                
                print("\nStarting spiral movement...")
                # Execute the spiral movement
                delta.move_in_spiral(
                    start_radius=start_radius,
                    end_radius=end_radius,
                    z_height=z_height,
                    steps=steps,
                    revolutions=revolutions,
                    delay=delay
                )
                
            elif choice == "3":
                # Figure-eight parameters
                width = float(input("Enter figure-eight width in mm (recommended: 200): ") or "200")
                height = float(input("Enter figure-eight height in mm (recommended: 100): ") or "100")
                z_height = float(input(f"Enter Z height in mm (recommended: {z_height:.1f}): ") or f"{z_height}")
                steps = int(input("Enter steps per cycle (recommended: 100): ") or "100")
                cycles = float(input("Enter number of cycles (recommended: 3): ") or "3")
                delay = float(input("Enter delay between steps in seconds (recommended: 0.05): ") or "0.05")
                
                print("\nStarting figure-eight movement...")
                # Execute the figure-eight movement
                delta.move_in_figure_eight(
                    width=width,
                    height=height,
                    z_height=z_height,
                    steps=steps,
                    cycles=cycles,
                    delay=delay
                )
                
            elif choice == "4":
                # 3D spiral parameters
                radius = float(input("Enter spiral radius in mm (recommended: 100): ") or "100")
                start_z = float(input(f"Enter starting Z height in mm (recommended: {delta.z_min * 0.7:.1f}): ") or f"{delta.z_min * 0.7}")
                end_z = float(input(f"Enter ending Z height in mm (recommended: {delta.z_max * 0.7:.1f}): ") or f"{delta.z_max * 0.7}")
                steps = int(input("Enter steps per revolution (recommended: 50): ") or "50")
                revolutions = float(input("Enter number of revolutions (recommended: 3): ") or "3")
                delay = float(input("Enter delay between steps in seconds (recommended: 0.05): ") or "0.05")
                
                print("\nStarting 3D spiral movement...")
                # Execute the 3D spiral movement
                delta.move_in_3d_spiral(
                    radius=radius,
                    start_z=start_z,
                    end_z=end_z,
                    steps=steps,
                    revolutions=revolutions,
                    delay=delay
                )
                
            else:
                print("Invalid choice. Exiting.")
            
        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            delta.close()
    else:
        print("Failed to connect to serial port. Check your connections and try again.")
