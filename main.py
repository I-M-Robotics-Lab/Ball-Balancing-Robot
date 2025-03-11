import threading
import time
import cv2
import math
import numpy as np
from camera import Camera
from controller import RobotController
from robotKinematics import RobotKinematics
from PID import PIDcontroller


# Shared variables
latest_frame = np.zeros((200, 150, 3), dtype=np.uint8)
lock = threading.Lock()
running = True

kp = 0.015
ki = 0.0
kd = 0.00865



#kp = 0.0134
#kd = 0.0024


kp = 0.0063 #0.0063   0.0046
ki = 0.00005 #0.00005
kd = 0.006025 #0.00595    0.00595



alpha = 0.65
beta = 0.3


# Initialize objects
cam = Camera()
model = RobotKinematics()
robot = RobotController(model, model.lp, model.l1, model.l2, model.lb)


PID = PIDcontroller(kp, ki, kd, alpha, beta, max_theta=model.maxtheta, conversion="tanh")

#Initialize Ball Position
x, y = 100, 75

def capture():

    global latest_frame
    while True:
        frame = cam.take_picture()
        with lock:
            latest_frame = frame 

def process():
    hz = 50
    global latest_frame, x, y
    while True:
        with lock:
            if latest_frame is None:
                continue 
            frame_copy = latest_frame.copy()
        
        loop_start = time.perf_counter()
        x, y = cam.coordinate(frame_copy)  
        x_t, y_t = (100, 75)  # Target position
        update_robot_pos(robot, model, PID, x_t, y_t, x, y)
        #cam.display_draw(frame_copy, (x,y))
        #print(f"Coordinates: {x, y}")
        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed
        if sleep_time > 0:
            #print(sleep_time)
            time.sleep(sleep_time)

def update_robot_pos(robotcontroller, robotkinematics, pidcontroller, x_t, y_t, x, y): #x_t, y_t: target position, x, y: current position, t: duration 

    theta, phi = pidcontroller.pid((x_t, y_t), (x, y))
    #print(theta, phi)
    #robotcontroller.Goto_time_spherical(theta, phi, 8.26, 0.02)
    robotcontroller.Goto_N_time_spherical(theta, phi, 8.26)



def pid_loop():
    hz = 30  # PID frequency
    while running:
        loop_start = time.perf_counter()
        x_t, y_t = (100, 75)  # Target position
        update_robot_pos(robot, model, PID, x_t, y_t, x, y)
        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed
        if sleep_time > 0:
            #print(sleep_time)
            time.sleep(sleep_time)
            
# Start threads
threading.Thread(target=capture, daemon=True).start()
threading.Thread(target=process, daemon=True).start()
time.sleep(2)
#threading.Thread(target=pid_loop).start()


# Keep running until manually stopped
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    running = False
    print("Exiting...")