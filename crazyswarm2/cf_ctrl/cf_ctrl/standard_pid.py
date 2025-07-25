
def saturation_fcn(a, min_value, max_value):
    if a > max_value:
        a = max_value
    if a < min_value:
        a = min_value
    return a




class PidControl(object):
    def __init__(self, sample_time, kp, ki, kd, constant):
        self.sample_time = sample_time
        self.kp = kp
        self.ki = ki
        self.ki_temp = 0
        self.kd = kd
        self.constant = constant
        self.desired_x = 0

        self.error_x = 0
        self.i_error_x = 0
        self.i_pause = False
        self.i_saturate = False
        self.i_saturate_min = 0
        self.i_saturate_max = 0

    def update_reference(self, desired_x):
        self.desired_x = desired_x

    def update_error(self, x):
        delayed_error_x = self.error_x
        self.error_x = self.desired_x - x
        d_error_x = (self.error_x - delayed_error_x) / self.sample_time
        if not self.i_pause:
            self.i_error_x = self.i_error_x + ((self.error_x + delayed_error_x) * self.sample_time / 2)
            if self.i_saturate:
                self.i_error_x = saturation_fcn(self.i_error_x, self.i_saturate_min, self.i_saturate_max)
        u_x = self.kp * self.error_x + self.ki_temp * self.i_error_x + self.kd * d_error_x + self.constant
        return u_x

    def integrator_enable(self):
        self.ki_temp = self.ki
        self.i_pause = False

    def integrator_disable(self):
        self.ki_temp = 0
        self.i_error_x = 0

    def integrator_pause(self):
        self.i_pause = True

    def integrator_saturation(self, min_value, max_value):
        self.i_saturate = True
        self.i_saturate_min = min_value
        self.i_saturate_max = max_value


class PidControlRaw(object):
    def __init__(self, kp, ki, kd, constant, i_saturate_min, i_saturate_max):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.constant = constant
        self.desired_x = 0
        self.desired_x_dot = 0
        self.error_x_int = 0
        self.i_saturate_min = i_saturate_min
        self.i_saturate_max = i_saturate_max

        self.time_delay = -1

    def update_reference(self, desired_x, desired_x_dot):
        self.desired_x = desired_x
        self.desired_x_dot = desired_x_dot

    def update_error(self, x, x_dot, abstime):
        dt = abstime - self.time_delay
        self.time_delay = abstime

        if dt > 0.2:
            dt = 0
            print('warning: sample time =', dt, 's > 0.2 s, integrator disable ')

        error_x = self.desired_x - x
        error_x_dot = self.desired_x_dot - x_dot
        self.error_x_int = saturation_fcn(self.error_x_int + error_x * dt, self.i_saturate_min, self.i_saturate_max)

        u_x = self.kp * error_x + self.ki * self.error_x_int + self.kd * error_x_dot + self.constant
        return u_x


class PidControlRaw2(object):
    def __init__(self, kp, ki, kd, constant, i_saturate_min, i_saturate_max):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.constant = constant
        self.desired_x = 0
        self.desired_x_dot = 0
        self.error_x_int = 0
        self.i_saturate_min = i_saturate_min
        self.i_saturate_max = i_saturate_max

        self.time_delay = -1

        self.integrator_enable = False
        self.I_term = 0

    def enable_integrator(self):
        self.integrator_enable = True

    def disable_integrator(self):
        self.integrator_enable = False

    def update_reference(self, desired_x, desired_x_dot):
        self.desired_x = desired_x
        self.desired_x_dot = desired_x_dot

    def update_error(self, x, x_dot, abstime):
        dt = abstime - self.time_delay
        self.time_delay = abstime

        if dt > 0.2:
            dt = 0
            # print('warning: sample time =', dt, 's > 0.2 s, integrator disable ')

        error_x = self.desired_x - x
        error_x_dot = self.desired_x_dot - x_dot
        if self.integrator_enable:
            self.error_x_int = saturation_fcn(self.error_x_int + error_x * dt, self.i_saturate_min, self.i_saturate_max)
        else:
            self.error_x_int = 0.0

        self.I_term = self.ki * self.error_x_int
        u_x = self.kp * error_x + self.I_term + self.kd * error_x_dot + self.constant
        return u_x
