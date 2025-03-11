import time
import cv2
from picamera2 import Picamera2, Preview
import numpy as np
from collections import deque
import threading

class Camera:
    #1640, 1232
    def __init__(self, resolution=(1640, 1232), format="RGB888"):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": resolution, "format": format}, controls={"FrameDurationLimits": (8333, 8333)})
        self.picam2.configure(config)

        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 50])
        self.gray_threshold = 60

        self.queue = deque(maxlen=16)
        self.queue.append((100, 75))  

        self.picam2.start()    

    def take_picture(self):
        image = self.picam2.capture_array()
        scale = 200.0 / image.shape[1]
        frame_resized = cv2.resize(image, (200, int(image.shape[0] * scale)))
        return frame_resized

    def display(self, image, window_name="Camera Output"):
        cv2.imshow(window_name, image)
        cv2.waitKey(1) 

    def display_draw(self, image, center, window_name="Tracked Output"):
        x, y = center
        cv2.line(image, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
        cv2.imshow(window_name, image)
        cv2.waitKey(1) 


    def terminate(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()

    def coordinate(self, image):
        
        prev_time = time.time()

        # Apply Gaussian blur.
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Convert from BGR to HSV.
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)
        frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)

        #Filter based on Darkness + HSV
        mask_hsv = cv2.inRange(frame_hsv, self.lower_black, self.upper_black)
        mask_gray = cv2.threshold(frame_gray, self.gray_threshold, 255, cv2.THRESH_BINARY_INV)[1]
        mask_combined = cv2.bitwise_or(mask_hsv, mask_gray)

        #Process Edges
        mask_eroded = cv2.erode(mask_combined, None, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, None, iterations=1)

        # --- Find Contours (circles)
        valid_detections = []
        contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Minimum Enclosing Circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)

            # Ignore small objects
            if radius < 5 or radius > 100:  # Adjust min/max radius based on expected size
                continue

            #Compute Circularity 4π(Area / Perimeter²)
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:  # Threshold to eliminate non-circular objects
                continue

            # Compute Aspect Ratio of Bounding Box
            x, y, w, h = cv2.boundingRect(contour)

            # If the contour passes all filters
            valid_detections.append((area, (int(x + w / 2), int(y + h / 2))))

        
        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]  
            self.queue.append(best_center) 
        else:
            if False and len(self.queue) >= 5 and self.queue[-1] == self.queue[-2] == self.queue[-3] == self.queue[-4] == self.queue[-5]:
                self.queue.append((100, 75))
            else:
                self.queue.append(self.queue[-1])

        return self.queue[-1]
        

if __name__ == "__main__":
    cam = Camera()

    try:
        while True:
            img = cam.take_picture()
            c = cam.coordinate(img)
            cam.display_draw(img, c)
            print(c)
            
            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.terminate()