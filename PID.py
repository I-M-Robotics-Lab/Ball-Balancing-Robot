import math
import time

class PIDcontroller:
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="linear"): #"linear" or "tanh"

        self.kp, self.ki, self.kd = kp, ki, kd
        self.alpha = alpha  #Exponential Filter: α⋅x + (1-α)⋅x_last
        self.beta = beta  #Coefficient for converting magnitude, either βx or tanh(βx)
        self.max_theta = max_theta

        if conversion == "linear":
            self.magnitude_convert = 1 #Linear
        elif conversion == "tanh":
            self.magnitude_convert = 0 #Tanh
        else:
            self.magnitude_convert = -1
 
        self.prev_out_x = 0.0
        self.prev_err_x = 0.0  
        self.prev_out_y = 0.0
        self.prev_err_y = 0.0

        self.sum_err_x = 0.0  #Integral
        self.sum_err_y = 0.0  #Integral
        
        self.last_time = None

    def pid(self, target, current):

        #dt
        new_time = time.perf_counter()
        dt = new_time - self.last_time if self.last_time is not None else 0.001

        #errors
        err_x = current[0] - target[0]
        err_y = current[1] - target[1]
        self.sum_err_x += err_x * dt
        self.sum_err_y += err_y * dt
        d_err_x = (err_x - self.prev_err_x) / dt if dt > 0 else 0
        d_err_y = (err_y - self.prev_err_y) / dt if dt > 0 else 0

        #output
        pid_x = self.kp * err_x + self.ki * self.sum_err_x + self.kd * d_err_x
        pid_y = self.kp * err_y + self.ki * self.sum_err_y + self.kd * d_err_y
        filtered_x = self.alpha * pid_x + (1 - self.alpha) * self.prev_out_x
        filtered_y = self.alpha * pid_y + (1 - self.alpha) * self.prev_out_y
        
        #Convert to spherical coordinates
        phi = math.degrees(math.atan2(filtered_y, filtered_x))
        if phi < 0:
            phi += 360
        r = math.sqrt(filtered_x**2 + filtered_y**2)
        if self.magnitude_convert == 1:
            theta = min(max(0, self.beta*r), self.max_theta)
        else:
            theta = max(0, 15*math.tanh(self.beta*r))


        self.prev_err_x = err_x
        self.prev_err_y = err_y
        self.prev_out_x = filtered_x
        self.prev_out_y = filtered_y
        self.last_time = new_time

        return theta, phi #in degrees

