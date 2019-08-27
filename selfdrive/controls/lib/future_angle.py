import numpy as np

class future_angle(object):
  def __init__(self, CP, rate=100):
    self.torque_count = int(CP.steerActuatorDelay * float(rate))
    self.torque_samples_outward = np.zeros(self.torque_count)
    self.torque_samples_inward = np.zeros(self.torque_count)
    self.angle_samples = np.zeros(self.torque_count)
    self.torque_gain_outward = np.zeros(15) + 1
    self.torque_gain_outward_count = np.zeros(15)
    self.torque_gain_inward = np.zeros(15) + 1
    self.torque_gain_inward_count = np.zeros(15)

    self.inverted = 0.
    self.frame = 0
    self.index_increment = 1
    self.spring_factor = 1.0
    self.outer_angle = 0.
    self.inner_angle = 0.
    self.deadzone = 1.5
  def update(self, v_ego, angle_steers, rate_steers, rate_steers_des, eps_torque, steer_override):

    rate_steers = 0.5 * rate_steers + 0.5 * rate_steers_des
    v_index = int(v_ego // 3)

    isOutward = (angle_steers > 0) == (self.inverted * eps_torque > 0) and (eps_torque != 0)
    isDeadzone = (isOutward and abs(angle_steers - self.inner_angle) < self.deadzone) or (not isOutward and abs(angle_steers - self.outer_angle) < self.deadzone)

    if not isDeadzone and not steer_override and abs(self.inverted) >= 1.0 and abs(angle_steers) > 1.0 and abs(rate_steers) > 1.0 and abs(eps_torque) > 1.0 and \
      max(max(self.torque_samples_inward), max(self.torque_samples_outward)) * min(min(self.torque_samples_inward), min(self.torque_samples_outward)) >= 0 and \
      max(self.angle_samples) * min(self.angle_samples) >= 0:

      if isOutward:
        self.torque_gain_outward_count[v_index] += self.index_increment
        self.torque_gain_outward[v_index] += ((abs(self.torque_samples_outward[self.frame % self.torque_count]) / abs(rate_steers)) - self.torque_gain_outward[v_index]) / min(1000,self.torque_gain_outward_count[v_index])
      else:
        self.torque_gain_inward_count[v_index] += self.index_increment
        self.torque_gain_inward[v_index] += ((abs(self.torque_samples_inward[self.frame % self.torque_count]) / abs(rate_steers)) - self.torque_gain_inward[v_index]) / min(1000,self.torque_gain_inward_count[v_index])

    elif not steer_override and abs(self.inverted) < 1.0 and abs(rate_steers * (self.torque_samples_inward[self.frame % self.torque_count] + self.torque_samples_outward[self.frame % self.torque_count])) > 0:
      if (rate_steers < 0) == ((self.torque_samples_inward[self.frame % self.torque_count] + self.torque_samples_outward[self.frame % self.torque_count]) < 0):
        self.inverted += 0.001
      else:
        self.inverted -= 0.001

    if isOutward:
      self.torque_samples_inward[self.frame % self.torque_count] = 0.0
      self.torque_samples_outward[self.frame % self.torque_count] = eps_torque
      if not isDeadzone:
        self.outer_angle = angle_steers
    else:
      self.torque_samples_outward[self.frame % self.torque_count] = 0.0
      self.torque_samples_inward[self.frame % self.torque_count] = eps_torque
      if not isDeadzone:
        self.inner_angle = angle_steers

    self.angle_samples[self.frame % self.torque_count] = angle_steers
    self.frame += 1

    if self.torque_gain_inward_count[v_index] >= 1000 and self.torque_gain_outward_count[v_index] >= 1000:
      self.index_increment = 0
      if not isDeadzone:
        advance_angle = self.torque_samples_inward.sum() / (self.torque_gain_inward[v_index] * self.spring_factor) + self.torque_samples_outward.sum() / self.torque_gain_outward[v_index]
      else:
        advance_angle = 0.
    else:
      self.index_increment = 1
      advance_angle = 0.
    #if (self.frame % 20 == 0): print(self.deadzone, self.spring_factor)

    return 0.01 * advance_angle * self.inverted
