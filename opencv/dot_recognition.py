import cv2 as cv
import numpy as np
# import serial 
import time

# Initialize serial connection (Adjust port & baud rate based on ESP32 configuration)
# ser = serial.Serial('COM3', 115200, timeout=1)
# time.sleep(2)  # Allow time for ESP32 to initialize
capture = cv.VideoCapture(0)

# Blob detector parameters
params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 10   # Adjust based on dot size
params.maxArea = 5000
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False

detector = cv.SimpleBlobDetector_create(params)

while True: 
    isTrue, frame = capture.read()
    
    if not isTrue:  # Check if frame is captured successfully
        print("Error: Couldn't read frame")
        break
    
    frame_flipped = cv.flip(frame, 1)
    frame_gray = cv.cvtColor(frame_flipped, cv.COLOR_BGR2GRAY)
    threshold, frame_thresh = cv.threshold(frame_gray, 100, 255, cv.THRESH_BINARY)
    
    # # Find contours
    # contours, hierarchy = cv.findContours(frame_thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # print(f'{len(contours)} contour(s) found!')
    
    keypoints = detector.detect(frame_thresh)
    keypoints = sorted(keypoints, key=lambda kp: (kp.pt[1], kp.pt[0]))
    
    # String to store all dot coordinates
    dot_data = ""    
    
    for i, kp in enumerate(keypoints, start=1):
        x, y = int(kp.pt[0]), int(kp.pt[1])  # Get center
        size = int(kp.size)  # Get size of detected blob
        label = f"DOT {i}"
        
        # print(f"Dot {i} found at ({x}, {y})")
        
        # Send dot coordinates via UART
        dot_data += f"DOT {i}: X={x}, Y={y}\n"
        
        #Draw on the frsme
        cv.putText(frame_flipped, label, (x - size,y - size - 4), cv.FONT_HERSHEY_COMPLEX, 0.5, (0,0,255), thickness=1)
        cv.rectangle(frame_flipped, (x - size, y - size), (x + size, y + size), (0, 255, 0), 2)   
    
    # Send data to ESP32 if any dots were detected
    # if dot_data:
    #     ser.write(dot_data.encode())  # Convert to bytes and send
    #     print("Sent to ESP32:\n", dot_data)
        
    # cv.imshow("Gray", frame_gray)
    # cv.imshow("Threshold", frame_thresh)
    cv.imshow("Detected Dots", frame_flipped)
    
    if cv.waitKey(20) & 0xFF == ord('d'):  # Press 'd' to exit
        break
    
capture.release()
cv.destroyAllWindows()
# ser.close()

cv.waitKey(0)