from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from common.numpy_fast import clip
from selfdrive.kegman_conf import kegman_conf


class LatControlPID(object):
  def __init__(self, CP):
    kegman_conf(CP)
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.frame = 0.
    self.angle_bias = 0.

  def live_tune(self, CP):
    self.frame += 1
    if self.frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      self.pid._k_i = ([0.], [float(kegman.conf['Ki'])])
      self.pid._k_p = ([0.], [float(kegman.conf['Kp'])])
      self.pid.k_f = (float(kegman.conf['Kf']))
      self.sat_time = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, VM, path_plan, live_params):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)
    self.angle_bias = float(clip(live_params.angleOffset - live_params.angleOffsetAverage, self.angle_bias - 0.002, self.angle_bias + 0.002))

    self.live_tune(CP)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      #self.angle_steers_des = path_plan.angleSteers  # get from MPC/PathPlanner
      #self.angle_steers_des = interp(sec_since_boot(), path_plan.mpcTimes, path_plan.mpcAngles)
      #angle_bias = live_params.angleOffset - live_params.angleOffsetAverage
      #self.angle_steers_des = path_plan.angleSteers + angle_bias  # get from MPC/PathPlanner
      self.angle_steers_des = path_plan.angleSteers  # get from MPC/PathPlanner
      if not steer_override:
        #self.angle_steers_des += live_params.angleOffset - live_params.angleOffsetAverage
        self.angle_steers_des += self.angle_bias
      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      deadzone = 1.0
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    # Reset sat_flat always, set it only if needed
    self.sat_flag = False

    # If PID is saturated, set time which it was saturated
    if self.pid.saturated and self.sat_time < 0.5:
      self.sat_time = sec_since_boot()

    # To save cycles, nest in sat_time check
    if self.sat_time > 0.5:
      # If its been saturated for 0.7 seconds then set flag
      if (sec_since_boot() - self.sat_time) > 0.7:
        self.sat_flag = True

      # If it is no longer saturated, clear the sat flag and timer
      if not self.pid.saturated:
        self.sat_time = 0.0

    return output_steer, float(self.angle_steers_des), pid_log
