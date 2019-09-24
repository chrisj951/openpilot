import json
import os

class kegman_conf():
  def __init__(self, CP=None):
    if CP is not None:
      self.type = CP.lateralTuning.which()
    self.conf = self.read_config(CP)
    if CP is not None:
      try:
        self.init_config(CP)
      except:
        self.conf = self.read_config(CP, True)
        self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if CP.lateralTuning.which() == 'pid':
      self.type = "pid"
      if self.conf['type'] == "-1":
        self.conf["type"] = "pid"
        write_conf = True
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['steerRatio'] == "-1":
        self.conf['steerRatio'] = str(round(CP.steerRatio,3))
        write_conf = True
      if self.conf['steerRateCost'] == "-1":
        self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
        write_conf = True
      if self.conf['rateFFGain'] == "-1":
        self.conf['rateFFGain'] = str(round(CP.lateralTuning.pid.rateFFGain,3))
        self.conf['deadzone'] = str(round(CP.lateralTuning.pid.deadzone,3))
        write_conf = True
    else:
      self.type = "indi"
      if self.conf['type'] == "-1":
        self.conf["type"] = "indi"
        write_conf = True
      if self.conf['timeConst'] == "-1":
        self.conf['type'] = "indi"
        self.conf['timeConst'] = str(round(CP.lateralTuning.indi.timeConstant,3))
        self.conf['actEffect'] = str(round(CP.lateralTuning.indi.actuatorEffectiveness,3))
        self.conf['outerGain'] = str(round(CP.lateralTuning.indi.outerLoopGain,3))
        self.conf['innerGain'] = str(round(CP.lateralTuning.indi.innerLoopGain,3))
        write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self, CP=None, Reset=False):
    self.element_updated = False
    try:
      with open('/data/openpilot/selfdrive/gernby.json', 'r') as f:
        base_config = json.load(f)
    except IOError:
        base_config = {  "tuneRev": "0.0.2","Kf": "-1","Ki": "-1","steerRatio":"-1","steerRateCost":"-1","Kp": "-1","dampTime": "-1","rateFFGain": "-1","reactMPC": "-1", \
        "type": "-1","dampMPC":"-1","polyReact":"-1","polyDamp":"-1","polyFactor":"-1","timeConst":"-1","actEffect":"-1", \
        "outerGain":"-1","innerGain":"-1","innerGain":"-1","outerGain":"-1","actEffect":"-1","timeConst":"-1","springFactor":"-1","deadzone":"0.25","simpledd":False}

    if Reset or not os.path.isfile('/data/kegman.json'):
      self.config = {"cameraOffset":"0.06", "lastTrMode":"1", "battChargeMin":"60", "battChargeMax":"70", "wheelTouchSeconds":"180", \
              "battPercOff":"25", "carVoltageMinEonShutdown":"11800", "brakeStoppingTarget":"0.25", "leadDistance":"5", "steerRatio":"-1", "steerRateCost":"-1", "deadzone":"0.25", "simpledd":False}
    else:
      with open('/data/kegman.json', 'r') as f:
        self.config = json.load(f)

    if "battPercOff" not in self.config:
      self.config.update({"battPercOff":"25"})
      self.config.update({"carVoltageMinEonShutdown":"11800"})
      self.config.update({"brakeStoppingTarget":"0.25"})
      self.element_updated = True

    if "liveParams" not in self.config:
      self.config.update({"liveParams":"1"})
      self.element_updated = True

    if "Kf" not in self.config:
      self.config.update({"Kf":"-1"})
      self.element_updated = True

    if "steerRatio" not in self.config:
      self.config.update({"steerRatio":"-1"})
      self.config.update({"steerRateCost":"-1"})
      self.element_updated = True

    if "leadDistance" not in self.config:
      self.config.update({"leadDistance":"5"})
      self.element_updated = True

    if "deadzone" not in self.config:
      self.config.update({"deadzone":"0.25"})
      self.element_updated = True

    if "simpledd" not in self.config:
      self.config.update({"simpledd":False})
      self.element_updated = True

    if "cameraOffset" not in self.config:
      self.config.update({"cameraOffset":"0.06"})
      self.element_updated = True

    if "tuneRev" not in self.config or self.config['tuneRev'] != base_config['tuneRev']:
      for key, value in base_config.iteritems():
        self.config.update({key: value})
        self.element_updated = True

    if ("type" not in self.config or self.config['type'] == "-1") and CP != None:
      self.config.update({"type":CP.lateralTuning.which()})
      print(CP.lateralTuning.which())
      self.element_updated = True

    if "dashIP" not in self.config:
      self.config.update({"dashIP":"tcp://gernstation.synology.me"})
      self.config.update({"dashCapture":"controlsState,liveParameters,gpsLocation"})
      self.element_updated = True

    if self.element_updated:
      print("updated")
      self.write_config(self.config)

    return self.config

  def write_config(self, config):
    try:
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
