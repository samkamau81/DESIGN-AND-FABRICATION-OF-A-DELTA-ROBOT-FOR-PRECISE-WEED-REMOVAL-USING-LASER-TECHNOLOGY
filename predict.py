import os
import cv2
import math
from ultralytics import YOLO

# ✅ Load the model
model_path = r"C:\Users\hp\OneDrive\Documents\5th_Year\Weed_Detector\yolo11n.pt"

if not os.path.exists(model_path):
    print(f"❌ Error: Model file '{model_path}' not found. Check the path!")
    exit()

model = YOLO(model_path)  # Load YOLO model
threshold = 0.5  # Confidence threshold

# ✅ Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Error: Could not open webcam.")
    exit()

# ✅ Set camera resolution
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("❌ Error: Failed to read frame from camera.")
        break

    # ✅ Perform YOLO detection
    results = model(frame)[0]

    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result

        if score > threshold:
            # ✅ Compute the center coordinates (x, y)
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2

            # ✅ Compute width, height, and radius
            width = x2 - x1
            height = y2 - y1
            radius = math.sqrt(width**2 + height**2) / 2  # Approximate radius

            # ✅ Print the coordinates and radius
            print(f"📍 Object {int(class_id)} at (X={x_center:.2f}, Y={y_center:.2f}), Radius ≈ {radius:.2f} pixels")

            # ✅ Draw bounding box, center point, and radius circle
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            cv2.circle(frame, (int(x_center), int(y_center)), int(radius), (255, 0, 0), 2)  # Blue radius circle
            cv2.putText(frame, f"R: {int(radius)} px", (int(x_center + 10), int(y_center + 10)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

    # ✅ Show the output
    cv2.imshow("Weed Detection", frame)

    # ✅ Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
