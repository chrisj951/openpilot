import numpy as np

class LQR(object):
  def __init__(self, CP, rate=100):
    self.scale = CP.lateralTuning.pid.lqr.scale
    self.torque_factor = 1.0

    self.A = np.array(CP.lateralTuning.pid.lqr.a).reshape((2,2))  #  [[0.               1.            ],
                                                                  #   [-0.22619643      1.21822268    ]]

    self.B = np.array(CP.lateralTuning.pid.lqr.b).reshape((2,1))  #  [[-1.92006585e-04                ],
                                                                  #   [3.95603032e-05                 ]]

    self.C = np.array(CP.lateralTuning.pid.lqr.c).reshape((1,2))  #  [[1.               0.            ]

    self.K = np.array(CP.lateralTuning.pid.lqr.k).reshape((1,2))  #  [[-110.73572306    451.22718255  ]]

    self.L = np.array(CP.lateralTuning.pid.lqr.l).reshape((2,1))  #  [[0.3233671                      ],
                                                                  #   [0.3185757                      ]]

    self.dc_gain = CP.lateralTuning.pid.lqr.dcGain

    #ret.lateralTuning.pid.lqr.scale = 1500.0
    #ret.lateralTuning.pid.lqr.a = [0., 1., -0.22619643, 1.21822268]
    #ret.lateralTuning.pid.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    #ret.lateralTuning.pid.lqr.c = [1., 0.]
    #ret.lateralTuning.pid.lqr.k = [-110.73572306, 451.22718255]
    #ret.lateralTuning.pid.lqr.l = [0.3233671, 0.3185757]
    #ret.lateralTuning.pid.lqr.dcGain = 0.002237852961363602

    #ret.lateralTuning.lqr.a = [0., 1., 0.98973369, -0.0087532]
    #ret.lateralTuning.lqr.b = [2.97807477e-04, -2.44433928e-05]
    #ret.lateralTuning.lqr.c = [1., 0.]
    #ret.lateralTuning.lqr.k = [-24.91209918, 165.59203037]
    #ret.lateralTuning.lqr.l = [-0.00758444, 0.12276381]
    #ret.lateralTuning.lqr.dcGain = 0.0048496652936339275

    self.x_hat = np.array([[0], [0]])

  def update(self, v_ego, angle_steers_des, angle_steers, eps_torque):
    torque_scale = self.torque_factor * (0.45 + v_ego / 60.0)**2  # Scale actuator model with speed

    # Update Kalman filter
    angle_steers_k = float(self.C.dot(self.x_hat))
    e = angle_steers - angle_steers_k
    self.x_hat = self.A.dot(self.x_hat) + self.B.dot(eps_torque / torque_scale) + self.L.dot(e)

    u_lqr = float(angle_steers_des / self.dc_gain - self.K.dot(self.x_hat))
    return torque_scale * u_lqr / self.scale
