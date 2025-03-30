import cv2 as cv
import numpy as np
import tempfile
import os

# Dummy dot recognition function (replace with your actual implementation)
def dot_recognition(image_path):
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
    
    dot_data = "" 
    # Draw detected blobs as red circles
    # im_with_keypoints = cv.drawKeypoints(image, keypoints, None, (0,0,255),
    #                                       cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # return im_with_keypoints
    
    for i, kp in enumerate(keypoints, start=1):
        x, y = int(kp.pt[0]), int(kp.pt[1])  # Get center
        size = int(kp.size)  # Get size of detected blob
        label = f"DOT {i}"
        
        print(f"Dot {i} found at ({x}, {y})")
        
        # Send dot coordinates via UART
        dot_data += f"DOT {i}: X={x}, Y={y}\n"
        
        #Draw on the frsme
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
