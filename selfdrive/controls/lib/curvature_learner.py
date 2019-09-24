from numpy import clip
import pickle
import csv

# HOW TO
# import this module to where you want to use it, such as from ```selfdrive.controls.lib.curvature_learner import CurvatureLearner```
# create the object ```self.curvature_offset = CurvatureLearner(debug=True)```
# call the update method ```self.curvature_offset.update(angle_steers - angle_offset, self.LP.d_poly)```
# The learned curvature offsets will save and load automatically
# by Zorrobyte
# version 1.0

class CurvatureLearner:
    def __init__(self, debug=False):
        self.offset = 0.
        self.learning_rate = 12000
        self.frame = 0
        self.debug = debug
        try:
            self.learned_offsets = pickle.load(open("/data/curvaturev1.p", "rb"))
        except (OSError, IOError) as e:
            self.learned_offsets = {
                "center": 0.,
                "left": 0.,
                "right": 0.
            }
            pickle.dump(self.learned_offsets, open("/data/curvaturev1.p", "wb"))

    def update(self, angle_steers=0., d_poly=None, v_ego=0.):
        if angle_steers > 0.5:
            if abs(angle_steers) < 3.:
                self.learned_offsets["center"] -= d_poly[3] / self.learning_rate
                self.offset = self.learned_offsets["center"]
            if abs(angle_steers) > 3.:
                self.learned_offsets["left"] -= d_poly[3] / self.learning_rate
                self.offset = self.learned_offsets["left"]
        elif angle_steers < -0.5:
            if abs(angle_steers) < 3.:
                self.learned_offsets["center"] += d_poly[3] / self.learning_rate
                self.offset = self.learned_offsets["center"]
            if abs(angle_steers) > 3.:
                self.learned_offsets["right"] += d_poly[3] / self.learning_rate
                self.offset = self.learned_offsets["right"]

        self.offset = clip(self.offset, -0.3, 0.3)
        self.frame += 1

        if self.frame == 12000:  # every 2 mins
            pickle.dump(self.learned_offsets, open("/data/curvaturev1.p", "wb"))
            self.frame = 0
        if self.debug:
            with open('/data/curvdebug.csv', 'a') as csv_file:
                csv_file_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_file_writer.writerow([self.learned_offsets, v_ego])
        return self.offset
