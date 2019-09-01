import numpy as np

class advance_angle(object):
  def __init__(self, CP):
    self.override_frame = 0
    self.steer_steps = np.zeros(int(CP.steerAdvanceCycles))
    self.steer_step_sum = 0.0

  def get_steer_advance(self, steer_advance, steer_step, steer_override, frame, CP):

    if steer_override or self.steer_step_sum * steer_step < 0:
      #if frame > self.override_frame: print("   skipping! ", frame)
      self.override_frame = frame + CP.steerAdvanceCycles
    #if frame == self.override_frame:
    #  print("  not skipping ", frame)
    advance_index = frame % CP.steerAdvanceCycles
    self.steer_step_sum -= self.steer_steps[advance_index]
    self.steer_steps[advance_index] = steer_step if frame > self.override_frame else 0
    self.steer_step_sum += self.steer_steps[advance_index]

    return (self.steer_step_sum - steer_advance) / float(CP.steerAdvanceCycles)
