"""Discrete PID controller with output saturation and anti-windup."""


class PID:
    def __init__(self, kp, ki, kd, dt, u_min, u_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.u_min, self.u_max = u_min, u_max
        self.integral = 0.0
        self.prev_meas = None

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # Derivative on measurement avoids a spike when the setpoint steps
        if self.prev_meas is None:
            d_meas = 0.0
        else:
            d_meas = (measurement - self.prev_meas) / self.dt
        self.prev_meas = measurement

        u_unsat = (self.kp * error
                   + self.ki * (self.integral + error * self.dt)
                   - self.kd * d_meas)
        u = min(max(u_unsat, self.u_min), self.u_max)

        # Anti-windup: only integrate when the output isn't saturated
        if u == u_unsat:
            self.integral += error * self.dt
        return u
