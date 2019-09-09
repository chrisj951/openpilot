from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import gernterp, interp, clip
import numpy as np
from selfdrive.controls.lib.latcontrol_helpers import model_polyfit, compute_path_pinv

kegman = kegman_conf()
CAMERA_OFFSET = float(kegman.conf['cameraOffset'])  # m from center car to camera

def calc_d_poly(l_poly, r_poly, g_poly, l_prob, r_prob, g_prob, lane_width):
  # This will improve behaviour when lanes suddenly widen
  lane_width = min(4.0, lane_width)
  l_prob = l_prob * gernterp(abs(l_poly[3]), [2, 2.5], [1.0, 0.0])
  r_prob = r_prob * gernterp(abs(r_poly[3]), [2, 2.5], [1.0, 0.0])

  path_from_left_lane = l_poly.copy()
  path_from_left_lane[3] -= lane_width / 2.0
  path_from_right_lane = r_poly.copy()
  path_from_right_lane[3] += lane_width / 2.0

  c_prob = l_prob + r_prob - l_prob * r_prob

  c_poly = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
  d_poly = (c_prob * c_poly + g_prob * g_poly) / (c_prob + g_prob)
  return c_poly, d_poly, c_prob


def calc_g_poly(l_poly, r_poly, c_poly, g_poly, l_prob, r_prob, c_prob, g_prob, lane_width):

  half_lane_width = lane_width / 2.0
  l_center = l_prob * (l_poly[3] - half_lane_width)
  r_center = r_prob * (r_poly[3] + half_lane_width)
  g_prob = 0.0001 + (c_prob**2 + g_prob**2) / (c_prob + g_prob + 0.0001)

  g_poly[3] = clip(0.0, g_poly[3] - 0.005, g_poly[3] + 0.005)
  g_poly[2] = clip(0.0, g_poly[2] - 0.001, g_poly[2] + 0.001)
  g_poly[1] = c_poly[1]
  g_poly[0] = c_poly[0]

  if l_center < 0.0 or r_center > 0.0:
    race_line_adjust = gernterp(abs(g_poly[3]), [0.0, 0.3], [0.0, 1.0])
    l_race_poly = (race_line_adjust * l_poly * l_prob + g_prob * g_poly) / (race_line_adjust * l_prob + g_prob + 0.0001)
    r_race_poly = (race_line_adjust * r_poly * r_prob + g_prob * g_poly) / (race_line_adjust * r_prob + g_prob + 0.0001)
    if l_center > -r_center:
      g_poly[3] = (r_center + g_prob * g_poly[3]) / (r_prob + g_prob + 0.0001)
    else:
      g_poly[3] = (l_center + g_prob * g_poly[3]) / (l_prob + g_prob + 0.0001)

    if c_poly[1] < 0.0:
      g_poly[2] = min(l_race_poly[2], r_race_poly[2], g_poly[2])
      g_poly[1] = min(l_race_poly[1], r_race_poly[1], c_poly[1])
    else:
      g_poly[2] = max(l_race_poly[2], r_race_poly[2], g_poly[2])
      g_poly[1] = max(l_race_poly[1], r_race_poly[1], c_poly[1])

  return g_poly, g_prob


class LanePlanner(object):
  def __init__(self):
    self.l_poly = [0., 0., 0., 0.]
    self.r_poly = [0., 0., 0., 0.]
    self.p_poly = [0., 0., 0., 0.]
    self.d_poly = np.array([0., 0., 0., 0.])
    self.g_poly = np.array([0., 0., 0., 0.])
    self.c_poly = np.array([0., 0., 0., 0.])

    self.lane_width_estimate = 2.85
    self.lane_width_certainty = 1.0
    self.lane_width = 2.85

    self.l_prob = 0.
    self.r_prob = 0.
    self.c_prob = 0.
    self.g_prob = 0.
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
    # only offset left and right lane lines; offsetting g_poly does not make sense
    self.l_poly[3] += CAMERA_OFFSET
    self.r_poly[3] += CAMERA_OFFSET

    # Find current lanewidth
    self.lane_width_certainty += 0.05 * (self.l_prob * self.r_prob - self.lane_width_certainty)
    current_lane_width = abs(self.l_poly[3] - self.r_poly[3])
    self.lane_width_estimate += 0.005 * (current_lane_width - self.lane_width_estimate)
    speed_lane_width = gernterp(controls.vEgo, [0., 31.], [2.85, 3.5])
    self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                      (1 - self.lane_width_certainty) * speed_lane_width

    self.g_poly, self.g_prob = calc_g_poly(self.l_poly, self.r_poly, self.d_poly, self.g_poly, self.l_prob, self.r_prob, self.c_prob, self.g_prob, self.lane_width)
    self.c_poly, self.d_poly, self.c_prob = calc_d_poly(self.l_poly, self.r_poly, self.g_poly, self.l_prob, self.r_prob, self.g_prob, self.lane_width)

  def update(self, md, controls):
    self.parse_model(md)
    self.update_lane(controls)
