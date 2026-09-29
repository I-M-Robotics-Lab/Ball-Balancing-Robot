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
        self.upper_black = np.array([179, 255, 50])
        self.gray_threshold = 60

        self.queue = deque(maxlen=16)
        self.queue.append((100, 75))  
        
        self._last_debug_time = 0.0

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

    def terminate(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()

    def coordinate(self, image):
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)

        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_RGB2HSV)
        frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_RGB2GRAY)

        mask_hsv = cv2.inRange(frame_hsv, self.lower_black, self.upper_black)
        mask_gray = cv2.threshold(frame_gray, self.gray_threshold, 255, cv2.THRESH_BINARY_INV)[1]
        mask_combined = cv2.bitwise_or(mask_hsv, mask_gray)

        mask_eroded = cv2.erode(mask_combined, None, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, None, iterations=1)
        
        if getattr(self, "debug_mask", False):
            # store for main thread to display
            self._dbg_mask = mask_dilated
            self._dbg_gray = frame_gray

        
        valid_detections = []
        contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_dbg = None

        for contour in contours:
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)
            if radius < 5 or radius > 100:
                continue

            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:
                continue
            
            if best_dbg is None or area > best_dbg[0]:
                best_dbg = (area, radius, circularity)

            x, y, w, h = cv2.boundingRect(contour)
            valid_detections.append((area, (int(x + w / 2), int(y + h / 2))))

        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]
            self.queue.append(best_center)
            return best_center[0], best_center[1], True
        else:
            now = time.time()
            last = getattr(self, "_last_debug_time", 0.0)
            if self.debug and now - last > 0.5:
                print(f"{now:.3f}  NO DETECTION  contours={len(contours)}")
                self._last_debug_time = now

        # Keep queue stable for drawing, but report invalid to control
        last_center = self.queue[-1]
        self.queue.append(last_center)
        return last_center[0], last_center[1], False



if __name__ == "__main__":
    cam = Camera()
    self.debug = False          # prints
    self.debug_mask = False     # show mask windows

    
    try:
        while True:
            img = cam.take_picture()
            x, y, valid = cam.coordinate(img)

            cam.display_draw(img, (x, y))

            if time.time() - getattr(cam, "_tprint", 0) > 0.5:
                print("center:", (x, y), "valid:", valid)
                cam._tprint = time.time()

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.terminate()

