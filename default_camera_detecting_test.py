import cv2
import numpy as np

def detect_circles(camera_index=0):
    cap = cv2.VideoCapture(camera_index)

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        edges = cv2.Canny(blurred, 100, 200)
        
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=200,
                                   param1=100, param2=30, minRadius=10, maxRadius=100)
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), r, (255, 255, 255), -1)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    M = cv2.moments(contours[0])
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    if center_x < frame.shape[1] // 2:
                        print("Fly left")
                    elif center_x > frame.shape[1] // 2:
                        print("Fly right")
                    if center_y < frame.shape[0] // 2:
                        print("Fly up")
                    elif center_y > frame.shape[0] // 2:
                        print("Fly down")
        
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Canny Edge Detection', edges)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cap.release()
    cv2.destroyAllWindows()

# apply
detect_circles(camera_index=0)

