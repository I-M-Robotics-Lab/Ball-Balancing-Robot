import board
import time
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
from robotKinematics import RobotKinematics
import math
import random

def clamp(value, lower=19, upper=90):
    return max(lower, min(value, upper))

class RobotController:
    def __init__(self, model, lp=7.125, l1=6.20, l2=4.50, lb=4.00):
        # Initialize robot kinematics
        #self.robot = RobotKinematics(lp=lp, l1=l1, l2=l2, lb=lb)
        self.robot = model
        
        # Initialize the ServoKit and assign servos
        self.Controller = ServoKit(channels=16)
        self.s1 = self.Controller.servo[13]
        self.s2 = self.Controller.servo[15]
        self.s3 = self.Controller.servo[14]

        # Configure servos
        for s in (self.s1, self.s2, self.s3):
            s.actuation_range = 270
            s.set_pulse_width_range(500, 2500)

        self.initialize()

    def initialize(self):
      
        print("Initializing ...")
        self.set_motor_angles(54, 54, 54)
        self.interpolate_time([19, 19, 19], duration=0.25)
        time.sleep(1)
        self.Goto_time_spherical(0, 0, 8.26, t=0.25)
        time.sleep(1)
        print("Initialized!")
    
    def set_motor_angles(self, theta1, theta2, theta3):
        
        # Calibrate offsets 
        self.s1.angle = clamp(theta1) - 4  # clamp(theta_n) + OFFSET_Sn
        self.s2.angle = clamp(theta2) 
        self.s3.angle = clamp(theta3) 

    def interpolate_time(self, target_angles, steps=100, duration=0.3, individual_durations=None):
 
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        if individual_durations is None:
            individual_durations = [duration] * 3
        max_duration = max(individual_durations)
        steps = max(1, int(max_duration / 0.01))
        for i in range(steps + 1):
            t = i * max_duration / steps
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, individual_durations)
            ]
            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)

    def interpolate_speed(self, target_angles, speed=30, individual_speeds=None):
   
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        if individual_speeds is None:
            individual_speeds = [speed] * 3
        durations = [
            abs(t - c) / s if s > 0 else 0 
            for c, t, s in zip(current_angles, target_angles, individual_speeds)
        ]
        max_duration = max(durations)
        steps = max(1, int(max_duration / 0.01))
        for i in range(steps + 1):
            t = i * max_duration / steps
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, durations)
            ]
            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.interpolate_time(target_angles, duration=t)

    def Goto_time_vector(self, a, b, c, h, t=0.5):
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.interpolate_time(target_angles, duration=t)

    def Goto_N_time_vector(self, a, b, c, h):
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.set_motor_angles(*target_angles)
    
    def Goto_N_time_spherical(self, theta, phi, h):
        
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        #print(theta, phi, target_angles)
        self.set_motor_angles(*target_angles)

    '''
    def Goto_Speed(self, alpha, beta, gamma, h, speed=240):

        gamma_ = max(math.sin(math.pi * 5 / 12), gamma)
        self.robot.solve_inverse_kinematics(alpha, beta, gamma_, h)
        target_angles = [
            radians_to_degrees(math.pi * 0.5 - self.robot.theta1),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta2),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta3)
        ]
        self.interpolate_speed(target_angles, speed=speed)

    def Goto_NOPOLATE(self, alpha, beta, gamma, h):
  
        gamma_ = max(math.sin(math.pi * 5 / 12), gamma)
        self.robot.solve_inverse_kinematics(alpha, beta, gamma_, h)
        self.set_motor_angles(
            radians_to_degrees(math.pi * 0.5 - self.robot.theta1),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta2),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta3)
        )
    '''


    def Dance1(self):
 
        self.Goto_time_vector(0.258819045103, 0, 0.965925826289, 8)
        for _ in range(3):
            for i in range(100):
                t = (2 * math.pi / 100) * i
                x = math.cos(math.pi * 5 / 12) * math.cos(t)
                y = math.cos(math.pi * 5 / 12) * math.sin(t)
                z = math.sin(math.pi * 5 / 12)
                print(x, y, z, math.sqrt(x**2 + y**2 + z**2))
                self.Goto_N_time_vector(x, y, z, 8)
                time.sleep(1/100)
        self.Goto_time_vector(0, 0, 1, 8)


if __name__ == "__main__":

    model = RobotKinematics()
    rc = RobotController(model)
    time.sleep(0.5)





