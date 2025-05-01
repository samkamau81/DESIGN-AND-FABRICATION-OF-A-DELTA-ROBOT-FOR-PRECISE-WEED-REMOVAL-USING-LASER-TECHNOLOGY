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
                 motor_min_rot: float = 0.0,
                 motor_max_rot: float = 0.5):
        
        # Delta robot parameters
        self.upper_arm_length = upper_arm_length  # Length of the upper arm (mm)
        self.lower_arm_length = lower_arm_length  # Length of the lower arm (mm)
        self.upper_base_radius = upper_base_radius  # Radius of the fixed base (mm)
        self.effector_radius = effector_radius  # Radius of the end effector (mm)
        self.initial_angle = initial_angle  # Initial angle between upper arm and horizontal (radians)
        
        # Motor rotation limits (mm)
        self.motor_min_rot = motor_min_rot
        self.motor_max_rot = motor_max_rot
        
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
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False
    
    def send_gcode(self, command):
        """Send G-code command to the CNC shield."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(f"{command}\n".encode())
            # Wait for response/acknowledgment
            response = self.serial_connection.readline().decode().strip()
            print(f"Sent: {command} | Response: {response}")
            return response
        else:
            print(f"Serial not connected. Command not sent: {command}")
            return None
    
    def inverse_kinematics(self, x: float, y: float, z: float) -> List[float]:
        """
        Calculate motor positions required to position end effector at (x,y,z).
        Returns a list of the three motor positions.
        """
        motor_positions = []
        
        for i in range(3):
            # Position of the i-th base joint
            base_x, base_y, base_z = self.base_joints[i]
            
            # Position of the i-th effector joint in global coordinates
            effector_x = x + self.effector_joints[i][0]
            effector_y = y + self.effector_joints[i][1]
            effector_z = z + self.effector_joints[i][2]
            
            # Vector from base joint to effector joint
            dx = effector_x - base_x
            dy = effector_y - base_y
            dz = effector_z - base_z
            
            # Distance between base and effector joints (squared)
            distance_squared = dx**2 + dy**2 + dz**2
            
            # Using the law of cosines to calculate the angle
            cos_angle = (self.upper_arm_length**2 + distance_squared - self.lower_arm_length**2) / (2 * self.upper_arm_length * math.sqrt(distance_squared))
            
            # Clamping cos_angle to avoid domain errors (due to floating point inaccuracies)
            cos_angle = max(-1.0, min(1.0, cos_angle))
            
            # Calculate the angle
            angle = math.acos(cos_angle)
            
            # Convert angle to motor position (simplified)
            # Here we map the angle to a motor position between min and max values
            # This is a simplified approach - in reality, you'd need more precise mapping
            # based on your specific mechanical setup
            angle_normalized = angle / math.pi  # normalize to [0,1] range
            motor_pos = self.motor_min_rot + angle_normalized * (self.motor_max_rot - self.motor_min_rot)
            
            motor_positions.append(motor_pos)
        
        return motor_positions
    
    def generate_random_position(self) -> Tuple[float, float, float]:
        """Generate a random position within the work envelope."""
        # Choose a random z value within the safe range
        z = random.uniform(self.z_min * 0.8, self.z_max * 0.8)  # Use 80% of range for safety
        
        # The allowed xy radius depends on the z height - simplified calculation
        max_radius = self.xy_radius_at_z0 * (1 - abs(z / self.z_min) * 0.5)
        
        # Choose random angle and radius
        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(0, max_radius)
        
        # Convert to cartesian coordinates
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        return x, y, z
    
    def generate_random_gcode(self) -> str:
        """Generate a G-code command for a random position within the work envelope."""
        x, y, z = self.generate_random_position()
        return f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F1000"
    
    def calculate_motor_positions(self, x: float, y: float, z: float) -> str:
        """Calculate motor positions and return as G-code string."""
        motor_pos = self.inverse_kinematics(x, y, z)
        return f"G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F1000"
    
    def run_random_movements(self, count: int = 10, delay: float = 2.0):
        """Generate and execute a series of random movements."""
        if not self.serial_connection:
            print("Serial connection not established. Connect first.")
            return
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        for i in range(count):
            x, y, z = self.generate_random_position()
            gcode = self.calculate_motor_positions(x, y, z)
            
            print(f"Movement {i+1}/{count}: Target position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            self.send_gcode(gcode)
            time.sleep(delay)
            
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
        gcode = self.calculate_motor_positions(start_x, start_y, z_height)
        self.send_gcode(gcode)
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
            
            # Calculate motor positions via inverse kinematics
            gcode = self.calculate_motor_positions(x, y, z_height)
            
            # Send the command
            self.send_gcode(gcode)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (total_steps // 10) == 0 or i == total_steps:
                progress = (i / total_steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("Circular movement completed")
    
    def move_in_smooth_circle(self, radius: float, z_height: float, feed_rate: float = 500, revolutions: float = 1.0):
        """
        Move the end effector in a smooth horizontal circular path using arc movement commands.
        This uses G2/G3 arc commands for smoother motion if your controller supports them.
        
        Args:
            radius (float): Radius of the circle in mm
            z_height (float): Fixed Z height for the circular movement
            feed_rate (float): Speed of movement in mm/min
            revolutions (float): Number of complete revolutions to perform
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
        
        print(f"Moving in a smooth circle of radius {radius}mm at height {z_height}mm")
        print(f"Performing {revolutions} revolutions at {feed_rate} mm/min")
        
        # Home the machine first
        self.send_gcode("G28")  # Home all axes
        time.sleep(1)
        
        # Set to absolute positioning mode
        self.send_gcode("G90")
        
        # Initial move to the starting position of the circle
        start_x = radius  # Start at (radius, 0)
        start_y = 0
        
        # Calculate motor positions for the starting point
        motor_pos = self.inverse_kinematics(start_x, start_y, z_height)
        self.send_gcode(f"G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F{feed_rate}")
        time.sleep(2)  # Give time to reach the starting position
        
        # For multiple revolutions, we'll do them one at a time
        for rev in range(int(revolutions)):
            print(f"Starting revolution {rev+1}/{int(revolutions)}")
            
            # For each revolution, we'll do a full circle using G2 (clockwise) or G3 (counterclockwise)
            # We need to convert the end points of the arc to motor positions
            
            # We're already at (radius, 0), and a full circle ends at the same point
            # For the CNC to recognize this as a full circle, we typically need I and J values
            # I is the X-offset to the center, J is the Y-offset to the center
            
            # Since we're starting at (radius, 0), the center is at (0, 0) from our position
            # So I = -radius (X offset to center) and J = 0 (Y offset to center)
            
            # We need to send the end point in motor coordinates
            end_x = radius  # Same as start for a full circle
            end_y = 0
            end_motor_pos = self.inverse_kinematics(end_x, end_y, z_height)
            
            # Use G3 for counterclockwise movement (standard for most CNC controllers)
            # Note: The I and J values are in the workspace coordinates, not motor coordinates
            self.send_gcode(f"G3 X{end_motor_pos[0]:.4f} Y{end_motor_pos[1]:.4f} Z{end_motor_pos[2]:.4f} I{-radius:.4f} J0 F{feed_rate}")
            
            # Wait for the move to complete
            # This is a simplified approach - in a real system, you might want to check for completion signals
            # Estimate time based on circle circumference and feed rate
            circumference = 2 * math.pi * radius  # mm
            estimated_time = (circumference / feed_rate) * 60  # seconds
            
            print(f"Estimated time for revolution: {estimated_time:.2f} seconds")
            time.sleep(estimated_time + 1)  # Add a small buffer
        
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
        gcode = self.calculate_motor_positions(start_x, start_y, z_height)
        self.send_gcode(gcode)
        time.sleep(1)  # Give time to reach the starting position
        
        # Generate and execute the spiral path
        for i in range(steps + 1):  # +1 to complete the path
            # Calculate the angle and radius for this step
            angle = 2 * math.pi * revolutions * i / steps
            radius = start_radius + (end_radius - start_radius) * i / steps
            
            # Calculate position on the spiral
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            # Calculate motor positions via inverse kinematics
            gcode = self.calculate_motor_positions(x, y, z_height)
            
            # Send the command
            self.send_gcode(gcode)
            
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
            
            # Calculate motor positions via inverse kinematics
            gcode = self.calculate_motor_positions(x, y, z_height)
            
            # Send the command
            self.send_gcode(gcode)
            
            # Wait for the specified delay
            time.sleep(delay)
            
            # Print progress every 10%
            if i % (total_steps // 10) == 0 or i == total_steps:
                progress = (i / total_steps) * 100
                print(f"Progress: {progress:.1f}% complete")
        
        print("Figure-eight movement completed")
    
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
        motor_min_rot=0.0,        # Min motor rotation
        motor_max_rot=0.5         # Max motor rotation
    )
    
    # Connect to the CNC shield via serial
    if delta.connect_serial():
        try:
            print("Choose a movement pattern:")
            print("1. Step-by-step circle (more precise)")
            print("2. Smooth circle (using G2/G3 commands - smoother but requires arc support in controller)")
            print("3. Spiral (inner to outer)")
            print("4. Figure-eight pattern")
            choice = input("Enter your choice (1-4): ")
            
            # Default Z height in the middle of the work envelope
            z_height = (delta.z_min + delta.z_max) / 2
            
            if choice == "1":
                # Step-by-step circle parameters
                circle_radius = float(input("Enter circle radius in mm (recommended: 100): ") or "100")
                z_height = float(input(f"Enter Z height in mm (recommended: {z_height:.1f}): ") or f"{z_height}")
                steps_per_rev = int(input("Enter steps per revolution (recommended: 60): ") or "60")
                num_revolutions = float(input("Enter number of revolutions (recommended: 3): ") or "3")
                step_delay = float(input("Enter delay between steps in seconds (recommended: 0.1): ") or "0.1")
                
                print("\nStarting step-by-step circular movement...")
                # Execute the circular movement
                delta.move_in_circle(
                    radius=circle_radius,
                    z_height=z_height,
                    steps=steps_per_rev,
                    revolutions=num_revolutions,
                    delay=step_delay
                )
                
            elif choice == "2":
                # Smooth circle parameters
                circle_radius = float(input("Enter circle radius in mm (recommended: 100): ") or "100")
                z_height = float(input(f"Enter Z height in mm (recommended: {z_height:.1f}): ") or f"{z_height}")
                feed_rate = float(input("Enter feed rate in mm/min (recommended: 600): ") or "600")
                num_revolutions = float(input("Enter number of revolutions (recommended: 3): ") or "3")
                
                print("\nStarting smooth circular movement...")
                # Execute the smooth circular movement
                delta.move_in_smooth_circle(
                    radius=circle_radius,
                    z_height=z_height,
                    feed_rate=feed_rate,
                    revolutions=num_revolutions
                )
                
            elif choice == "3":
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
                
            elif choice == "4":
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
                
            else:
                print("Invalid choice. Exiting.")
            
        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            delta.close()
    else:
        # If serial connection fails, just demonstrate some positions
        print("Serial connection failed. Demonstrating path points:")
        
        # Choose a default pattern to demonstrate
        print("Choose a pattern to demonstrate:")
        print("1. Circle")
        print("2. Spiral")
        print("3. Figure-eight")
        demo_choice = input("Enter your choice (1-3): ") or "1"
        
        # Default Z height in the middle of the work envelope
        z_height = (delta.z_min + delta.z_max) / 2
        steps = 12  # Number of points to show for demo
        
        if demo_choice == "2":
            # Spiral demo
            start_radius = 20.0
            end_radius = 150.0
            
            print(f"\nDemonstrating points along a spiral from radius {start_radius}mm to {end_radius}mm at height {z_height:.1f}mm:")
            for i in range(steps):
                # Calculate point along spiral
                angle = 2 * math.pi * i / steps
                radius = start_radius + (end_radius - start_radius) * i / steps
                
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                
                # Calculate motor positions
                motor_pos = delta.inverse_kinematics(x, y, z_height)
                
                print(f"Spiral point {i+1}/{steps}: End effector at X={x:.2f}, Y={y:.2f}, Z={z_height:.2f}")
                print(f"Motor positions: X={motor_pos[0]:.4f}, Y={motor_pos[1]:.4f}, Z={motor_pos[2]:.4f}")
                print(f"G-code: G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F1000")
                print("-" * 40)
                
        elif demo_choice == "3":
            # Figure-eight demo
            width = 200.0
            height = 100.0
            
            print(f"\nDemonstrating points along a figure-eight with width {width}mm and height {height}mm at z={z_height:.1f}mm:")
            for i in range(steps):
                # Calculate point along figure-eight
                t = 2 * math.pi * i
                            # Figure-eight demo
            width = 200.0
            height = 100.0
            
            print(f"\nDemonstrating points along a figure-eight with width {width}mm and height {height}mm at z={z_height:.1f}mm:")
            for i in range(steps):
                # Calculate point along figure-eight
                t = 2 * math.pi * i / steps
                
                x = (width/2) * math.sin(t)
                y = (height/2) * math.sin(t) * math.cos(t)
                
                # Calculate motor positions
                motor_pos = delta.inverse_kinematics(x, y, z_height)
                
                print(f"Figure-eight point {i+1}/{steps}: End effector at X={x:.2f}, Y={y:.2f}, Z={z_height:.2f}")
                print(f"Motor positions: X={motor_pos[0]:.4f}, Y={motor_pos[1]:.4f}, Z={motor_pos[2]:.4f}")
                print(f"G-code: G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F1000")
                print("-" * 40)
                
        else:
            # Default to circle demo
            radius = 100.0
            
            print(f"\nDemonstrating points along a circle of radius {radius}mm at height {z_height:.1f}mm:")
            for i in range(steps):
                # Calculate point along circle
                angle = 2 * math.pi * i / steps
                
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                
                # Calculate motor positions
                motor_pos = delta.inverse_kinematics(x, y, z_height)
                
                print(f"Circle point {i+1}/{steps}: End effector at X={x:.2f}, Y={y:.2f}, Z={z_height:.2f}")
                print(f"Motor positions: X={motor_pos[0]:.4f}, Y={motor_pos[1]:.4f}, Z={motor_pos[2]:.4f}")
                print(f"G-code: G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F1000")
                print("-" * 40)


