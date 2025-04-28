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
    
    def connect_serial(self, port='/dev/ttyUSB0', baud_rate=115200):
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
            print("Starting random movement sequence...")
            # Generate and execute 20 random movements with 3 seconds between each
            delta.run_random_movements(count=20, delay=3.0)
        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            delta.close()
    else:
        # If serial connection fails, just demonstrate some random positions
        print("Serial connection failed. Demonstrating random positions:")
        for i in range(5):
            x, y, z = delta.generate_random_position()
            motor_pos = delta.inverse_kinematics(x, y, z)
            print(f"Random position {i+1}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            print(f"Motor positions: A={motor_pos[0]:.4f}, B={motor_pos[1]:.4f}, C={motor_pos[2]:.4f}")
            print(f"G-code: G1 X{motor_pos[0]:.4f} Y{motor_pos[1]:.4f} Z{motor_pos[2]:.4f} F1000")
            print("-" * 40)