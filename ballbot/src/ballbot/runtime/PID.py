import math
import time

class PIDcontroller:
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="linear"):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.alpha = alpha
        self.beta = beta
        self.max_theta = max_theta

        if conversion == "linear":
            self.magnitude_convert = 1
        elif conversion == "tanh":
            self.magnitude_convert = 0
        else:
            self.magnitude_convert = -1

        self.prev_out_x = 0.0
        self.prev_err_x = 0.0
        self.prev_out_y = 0.0
        self.prev_err_y = 0.0

        self.sum_err_x = 0.0
        self.sum_err_y = 0.0

        self.last_time = None

        # Anti-windup / dropout behavior
        self.I_MAX = 2000.0
        self.dropout_decay = 0.90  # 0.9 -> ramps toward 0 in ~10 cycles

    def pid(self, target, current, valid=True):
        new_time = time.perf_counter()
        dt = (new_time - self.last_time) if self.last_time is not None else (1/50)
        dt = max(0.005, min(0.05, dt))  # clamp dt for stable D/I

        if not valid:
            # Don’t integrate/derivative on junk input. Ramp output toward 0.
            self.prev_out_x *= self.dropout_decay
            self.prev_out_y *= self.dropout_decay
            self.prev_err_x = 0.0
            self.prev_err_y = 0.0
            self.sum_err_x = 0.0
            self.sum_err_y = 0.0
            self.last_time = new_time

            # When theta=0, phi is irrelevant; keep 0 for determinism
            return 0.0, 0.0
           
        # Errors (keeping your sign convention)
        err_x = current[0] - target[0]
        err_y = current[1] - target[1]

        # Integral (once!)
        self.sum_err_x += err_x * dt
        self.sum_err_y += err_y * dt
        self.sum_err_x = max(-self.I_MAX, min(self.I_MAX, self.sum_err_x))
        self.sum_err_y = max(-self.I_MAX, min(self.I_MAX, self.sum_err_y))

        # Derivative
        d_err_x = (err_x - self.prev_err_x) / dt
        d_err_y = (err_y - self.prev_err_y) / dt

        # PID output
        pid_x = self.kp * err_x + self.ki * self.sum_err_x + self.kd * d_err_x
        pid_y = self.kp * err_y + self.ki * self.sum_err_y + self.kd * d_err_y

        # Exponential filter on output
        filtered_x = self.alpha * pid_x + (1 - self.alpha) * self.prev_out_x
        filtered_y = self.alpha * pid_y + (1 - self.alpha) * self.prev_out_y

        # Convert to spherical coordinates
        phi = math.degrees(math.atan2(filtered_y, filtered_x))
        if phi < 0:
            phi += 360.0

        r = math.sqrt(filtered_x**2 + filtered_y**2)
        if self.magnitude_convert == 1:
            theta = min(max(0.0, self.beta * r), self.max_theta)
        else:
            theta = max(0.0, self.max_theta * math.tanh(self.beta * r))

        # State update
        self.prev_err_x = err_x
        self.prev_err_y = err_y
        self.prev_out_x = filtered_x
        self.prev_out_y = filtered_y
        self.last_time = new_time

        return theta, phi
