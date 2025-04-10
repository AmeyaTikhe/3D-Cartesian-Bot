import cv2 as cv
import numpy as np
import tempfile
import os
import serial
import time

# Initialize Serial Port for ESP32
SERIAL_PORT = "COM6"  # Change for Windows: "COM3"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to initialize
    print(f"Connected to ESP32 on {SERIAL_PORT}")
except Exception as e:
    print("Error opening serial port:", e)
    ser = None  # Avoid crashing if serial port isn't found
    
def dot_recognition(image_path):
    global ser  # Use global serial object
    image = cv.imread(image_path)
    # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Simple blob detection parameters
    params = cv.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 5000
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    detector = cv.SimpleBlobDetector_create(params)
    keypoints = detector.detect(frame_thresh)
    keypoints = sorted(keypoints, key=lambda kp: (kp.pt[1], kp.pt[0]))
    
    if not keypoints:
        print("No dots detected.")
        return

    # Prepare the coordinate string for ESP32
    dot_data = ";".join(f"{int(kp.pt[0])},{int(kp.pt[1])}" for kp in keypoints) + "\n"

    print("Detected dots:", dot_data.strip())

    # Send the coordinate string via UART
    if ser:
        ser.write(dot_data.encode())
        print(f"Sent to ESP32: {dot_data.strip()}")
    
    for i, kp in enumerate(keypoints, start=1):
        x, y = int(kp.pt[0]), int(kp.pt[1])  # Get center
        size = int(kp.size)  # Get size of detected blob
        label = f"DOT {i}"
        
        print(f"Dot {i} found at ({x}, {y})")
                
        #Draw on the frame
        cv.putText(frame_flipped, label, (x - size,y - size - 4), cv.FONT_HERSHEY_COMPLEX, 0.5, (0,0,255), thickness=1)
        cv.rectangle(frame_flipped, (x - size, y - size), (x + size, y + size), (0, 255, 0), 2)   
        
    return

# Initialize camera
capture = cv.VideoCapture(0)
if not capture.isOpened():
    print("Cannot open camera")
    exit()

temp_img_path = None

while True:
    isTrue, frame = capture.read()
    
    if not isTrue:  # Check if frame is captured successfully
        print("Error: Couldn't read frame")
        break
    
    frame_flipped = cv.flip(frame, 1)
    frame_gray = cv.cvtColor(frame_flipped, cv.COLOR_BGR2GRAY)
    threshold, frame_thresh = cv.threshold(frame_gray, 100, 255, cv.THRESH_BINARY)
    
    cv.imshow("Live Feed - Press 'p' to capture, 'd' to exit", frame_flipped)
    key = cv.waitKey(1) & 0xFF

    if key == ord('p'):
        # Create a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.jpg') as tmp:
            temp_img_path = tmp.name
            cv.imwrite(temp_img_path, frame_flipped)
            print(f"Captured and saved to {temp_img_path}")

        # Run dot recognition
        dot_recognition(temp_img_path)
        cv.imshow("Dot Recognition Result", frame_flipped)

    elif key == ord('d'):
        print("Shutting down...")
        break

capture.release()
cv.destroyAllWindows()

# Cleanup temporary file
if temp_img_path and os.path.exists(temp_img_path):
    os.remove(temp_img_path)
    print(f"Deleted temporary file: {temp_img_path}")
