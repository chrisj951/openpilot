from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp
import numpy as np
from selfdrive.controls.lib.latcontrol_helpers import model_polyfit, compute_path_pinv

kegman = kegman_conf()
CAMERA_OFFSET = float(kegman.conf['cameraOffset'])  # m from center car to camera

def calc_d_poly(l_poly, r_poly, p_poly, l_prob, r_prob, lane_width):
  # This will improve behaviour when lanes suddenly widen
  lane_width = min(4.0, lane_width)
  l_prob = l_prob * interp(abs(l_poly[3]), [2, 2.5], [1.0, 0.0])
  r_prob = r_prob * interp(abs(r_poly[3]), [2, 2.5], [1.0, 0.0])

  path_from_left_lane = l_poly.copy()
  path_from_left_lane[3] -= lane_width / 2.0
  path_from_right_lane = r_poly.copy()
  path_from_right_lane[3] += lane_width / 2.0

  lr_prob = l_prob + r_prob - l_prob * r_prob

  d_poly_lane = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
  return lr_prob * d_poly_lane + (1.0 - lr_prob) * p_poly


class LanePlanner(object):
  def __init__(self):
    self.l_poly = [0., 0., 0., 0.]
    self.r_poly = [0., 0., 0., 0.]
    self.p_poly = [0., 0., 0., 0.]
    self.d_poly = [0., 0., 0., 0.]

    self.lane_width_estimate = 3.7
    self.lane_width_certainty = 1.0
    self.lane_width = 3.7

    self.l_prob = 0.
    self.r_prob = 0.
    self.lr_prob = 0.
    self.race_factor = 0.

    self._path_pinv = compute_path_pinv()
    self.x_points = np.arange(50)

  def parse_model(self, md):
    if len(md.leftLane.poly):
      self.l_poly = np.array(md.leftLane.poly)
      self.r_poly = np.array(md.rightLane.poly)
      self.p_poly = np.array(md.path.poly)
    else:
      self.l_poly = model_polyfit(md.leftLane.points, self._path_pinv)  # left line
      self.r_poly = model_polyfit(md.rightLane.points, self._path_pinv)  # right line
      self.p_poly = model_polyfit(md.path.points, self._path_pinv)  # predicted path
    self.l_prob = md.leftLane.prob  # left line prob
    self.r_prob = md.rightLane.prob  # right line prob

  def update_lane(self, controls):
    # only offset left and right lane lines; offsetting p_poly does not make sense
    self.l_poly[3] += CAMERA_OFFSET
    self.r_poly[3] += CAMERA_OFFSET

    self.lr_prob = self.l_prob + self.r_prob - self.l_prob * self.r_prob

    # Find current lanewidth
    self.lane_width_certainty += 0.05 * (self.l_prob * self.r_prob - self.lane_width_certainty)
    current_lane_width = abs(self.l_poly[3] - self.r_poly[3])
    self.lane_width_estimate += 0.005 * (current_lane_width - self.lane_width_estimate)
    speed_lane_width = interp(controls.vEgo, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                      (1 - self.lane_width_certainty) * speed_lane_width

    d_poly = calc_d_poly(self.l_poly, self.r_poly, self.p_poly, self.l_prob, self.r_prob, self.lane_width)

    '''self.p_poly = d_poly.copy()

    if controls.lateralControlState.which() == "pidState":
      torque_request = controls.lateralControlState.pidState.output
    elif controls.lateralControlState.which() == "lqrState":
      torque_request = controls.lateralControlState.lqrState.output
    else:
      torque_request = controls.lateralControlState.indiState.output
    self.race_factor += 0.05 * (abs(torque_request) - self.race_factor)
    curv_factor = abs(d_poly[1] / 0.001)

    if (d_poly[3] <= 0) != (self.d_poly[1] <= 0) and (self.d_poly[3] <= 0) != (torque_request <= 0): # and abs(d_poly[3]) <= abs(self.d_poly[3]):
      self.p_poly[3] *= max(0.0, 1.0 - self.race_factor * curv_factor)
    #else:
    #  self.p_poly[3] += (d_poly[3] - self.d_poly[3]) * self.race_factor * 10.0

    #if (d_poly[2] <= 0) != (self.d_poly[1] <= 0) and (self.d_poly[2] <= 0) != (torque_request <= 0): # and abs(d_poly[2]) <= abs(self.d_poly[2]):
    #  self.p_poly[2] *= max(0.5, 1.0 - self.race_factor * curv_factor)
    #else:
    #  self.p_poly[2] += (d_poly[2] - self.d_poly[2]) * self.race_factor * 10.0
    '''

    self.d_poly = d_poly

  def update(self, md, controls):
    self.parse_model(md)
    self.update_lane(controls)
