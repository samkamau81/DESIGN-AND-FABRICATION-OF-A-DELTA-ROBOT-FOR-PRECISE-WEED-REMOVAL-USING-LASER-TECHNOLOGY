import cv2
import numpy as np
import serial
import time
import torch
import math

# ---------------------- INITIALIZATION ----------------------

# Serial communication with Arduino (modify port as needed)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Allow time to establish connection

# Load YOLOv11 model (modify path if needed)
model = torch.hub.load('ultralytics/yolov5', 'custom', path='weed_model.pt')

# Initialize camera
cap = cv2.VideoCapture(0)

# Delta robot parameters (adjust for your setup)
DELTA_BASE_RADIUS = 50  # Distance from center to base joints
DELTA_END_RADIUS = 20   # Distance from center to end effector joints
BICEP_LENGTH = 100      # Upper link (bicep) length
FOREARM_LENGTH = 120    # Lower link (forearm) length

# ---------------------- OBJECT DETECTION ----------------------

def detect_weeds(frame):
    """
    Detect weeds using YOLO and return list of (x, y, radius) coordinates.
    """
    results = model(frame)
    detections = results.xyxy[0]  # Bounding boxes: (x_min, y_min, x_max, y_max, conf, class)

    weeds = []
    for det in detections:
        x_min, y_min, x_max, y_max, conf, cls = det
        if conf > 0.5:  # Confidence threshold
            x_center = int((x_min + x_max) / 2)
            y_center = int((y_min + y_max) / 2)
            radius = int((x_max - x_min) / 2)  # Approximate weed radius
            weeds.append((x_center, y_center, radius))

    return weeds

# ---------------------- INVERSE KINEMATICS ----------------------

def inverse_kinematics(x, y, z=0):
    """
    Compute inverse kinematics for the delta robot.
    Returns base angles (theta1, theta2, theta3) and elbow angles (phi1, phi2, phi3).
    """
    # Define base joint positions relative to the robot's center
    base_positions = [
        (DELTA_BASE_RADIUS, 0), 
        (-DELTA_BASE_RADIUS / 2, DELTA_BASE_RADIUS * math.sqrt(3) / 2),
        (-DELTA_BASE_RADIUS / 2, -DELTA_BASE_RADIUS * math.sqrt(3) / 2)
    ]

    theta = []
    phi = []

    for bx, by in base_positions:
        dx = x - bx
        dy = y - by
        dz = z

        # Distance from base joint to target
        D = math.sqrt(dx**2 + dy**2 + dz**2)

        # Ensure point is reachable
        if D > (BICEP_LENGTH + FOREARM_LENGTH) or D < abs(BICEP_LENGTH - FOREARM_LENGTH):
            raise ValueError("Point out of reach")

        # Solve for base rotation angle (theta)
        theta_i = math.atan2(dy, dx) * 180 / math.pi

        # Solve for elbow angle (phi) using cosine rule
        cos_phi = (BICEP_LENGTH**2 + D**2 - FOREARM_LENGTH**2) / (2 * BICEP_LENGTH * D)
        phi_i = math.acos(cos_phi) * 180 / math.pi

        theta.append(theta_i)
        phi.append(phi_i)

    return theta, phi

# ---------------------- G-CODE GENERATION ----------------------

def generate_gcode(x, y, r):
    """
    Generate G-Code to move delta robot to weed location, control the laser with PWM,
    and perform a spiral inward motion to burn the weed.
    """
    # Compute inverse kinematics
    theta, phi = inverse_kinematics(x, y)

    gcode = [
        f"G1 A{theta[0]:.2f} B{theta[1]:.2f} C{theta[2]:.2f} F1000",  # Move base joints
        f"G1 D{phi[0]:.2f} E{phi[1]:.2f} F{phi[2]:.2f} F1000",  # Move elbow joints
        "M3 S255",  # Activate laser at full power (PWM max)
        "G4 P1",  # Wait 1 second for full power stabilization
    ]

    # Spiral inward from max radius to center
    for radius in range(r, 0, -2):
        theta, phi = inverse_kinematics(x, y)
        gcode.append(f"G1 A{theta[0]:.2f} B{theta[1]:.2f} C{theta[2]:.2f} F500")  # Adjust base
        gcode.append(f"G1 D{phi[0]:.2f} E{phi[1]:.2f} F{phi[2]:.2f} F500")  # Adjust elbow

    gcode.append("M5")  # Turn off laser (PWM = 0)
    return gcode

# ---------------------- SERIAL COMMUNICATION ----------------------

def send_gcode_to_arduino(gcode):
    """
    Send the generated G-Code commands to the Arduino via serial communication.
    """
    for command in gcode:
        ser.write((command + '\n').encode())
        time.sleep(0.1)  # Allow execution time

# ---------------------- MAIN LOOP ----------------------

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    weeds = detect_weeds(frame)

    for (x, y, r) in weeds:
        print(f"Weed detected at ({x}, {y}) with radius {r}")
        gcode = generate_gcode(x, y, r)
        send_gcode_to_arduino(gcode)

    # Show detected weeds for debugging
    for (x, y, r) in weeds:
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)

    cv2.imshow("Weed Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
