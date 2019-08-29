import numpy as np

class eps_rate_gain(object):
  def __init__(self, CP, rate=100):
    self.torque_count = int(CP.steerActuatorDelay * float(rate)) - 1
    self.torque_samples = np.zeros(self.torque_count)
    self.angle_samples = np.zeros(self.torque_count)
    self.rate_samples = np.zeros(self.torque_count)

    self.frame = 0
    self.outer_angle = 0.
    self.inner_angle = 0.
    self.prev_angle = 0.
    self.torque_sum = 0.
    self.torque_rate_factor = 0.
    self.deadzone = 1.0
    self.spring_factor = 0.0
    self.override_frame = 0
    self.prev_override = False

  def update(self, v_ego, angle_steers, rate_steers_des, eps_torque, steer_override, saturated):

    if steer_override:
        self.override_frame = self.frame + self.torque_count
    if abs(angle_steers) > abs(self.prev_angle) and abs(angle_steers - self.inner_angle) > self.deadzone:
        self.outer_angle = angle_steers
    elif abs(angle_steers) < abs(self.prev_angle) and abs(angle_steers - self.outer_angle) > self.deadzone:
        self.inner_angle = angle_steers

    notDeadzone = v_ego > 10.0 and abs(angle_steers) >= self.deadzone and self.frame > self.override_frame and \
        ((abs(angle_steers - self.inner_angle) > self.deadzone and abs(self.angle_samples[self.frame % self.torque_count]) > abs(self.inner_angle) or \
        (abs(angle_steers - self.outer_angle) > self.deadzone) and abs(self.angle_samples[self.frame % self.torque_count]) < abs(self.outer_angle)))

    self.torque_sum += eps_torque
    if notDeadzone and not saturated:
        if abs(self.torque_sum) > 0 and abs(angle_steers) < 30 and abs(rate_steers_des) < 20:
            self.torque_rate_factor += 0.01 * (((angle_steers - self.angle_samples[self.frame % self.torque_count]) / self.torque_sum) - self.torque_rate_factor)
    self.torque_sum -= self.torque_samples[self.frame % self.torque_count]

    if notDeadzone:
        self.advance_angle = self.spring_factor * self.torque_sum * self.torque_rate_factor
    else:
        self.advance_angle = 0.

    self.rate_samples[self.frame % self.torque_count] = 100.0 * eps_torque * self.torque_rate_factor
    self.torque_samples[self.frame % self.torque_count] = eps_torque
    self.angle_samples[self.frame % self.torque_count] = angle_steers
    self.prev_angle = angle_steers
    self.prev_override = steer_override
    self.frame += 1

    if self.frame % 10 == 0: print(" %0.2f   %0.2f   %0.7f" % (self.advance_angle, self.rate_samples[self.frame % self.torque_count], self.torque_rate_factor))
    return float(self.advance_angle), float(self.rate_samples[self.frame % self.torque_count])
