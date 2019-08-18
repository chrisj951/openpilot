from collections import namedtuple
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip
from selfdrive.car import create_gas_command
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import AH, CruiseButtons, CAR, CruiseSettings
from selfdrive.can.packer import CANPacker
from selfdrive.kegman_conf import kegman_conf

kegman = kegman_conf()


def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small oscillations within this value

  #*** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.ACURA_ILX, CAR.CRV)) and brake > 0.0:
    brake += 0.15

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


def process_hud_alert(hud_alert, vEgo):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  ldw_display = 0
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  elif hud_alert == AH.LDW and vEgo >= 25:         # LDW
    ldw_display = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert, ldw_display


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "fcw", "acc_alert", "steer_required", "dashed_lanes", "ldw"])

class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.BRAKE_MAX = 1024//4
    if car_fingerprint in (CAR.ACURA_ILX):
      self.STEER_MAX = 0xF00
    elif car_fingerprint in (CAR.CRV, CAR.ACURA_RDX):
      self.STEER_MAX = 0x3e8  # CR-V only uses 12-bits and requires a lower value (max value from energee)
    elif car_fingerprint in (CAR.ODYSSEY_CHN):
      self.STEER_MAX = 0x7FFF
    else:
      self.STEER_MAX = 0x1000
    self.STEER_DELTA_UP = 27           # torque increase per refresh, 1.517s to max
    self.STEER_DELTA_DOWN = 68         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 1200 # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1   # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1       # from dbc

class CarController(object):
  def __init__(self, dbc_name, car_fingerprint):
    self.apply_steer_last = 0
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False
    self.prev_lead_distance = 0.0 # a non-linear value
    self.lead_distance_counter = 1.0 # seconds since last update
    self.lead_distance_counter_prev = 1
    self.rough_lead_speed = 0.0 #delta m/s
    self.desiredTR = 0 # the desired distance bar
    self.params = CarControllerParams(car_fingerprint)

  def honda_bosch_to_meters(self, lead_distance):
    if lead_distance < 100:
      return (0.1 * lead_distance) * 0.9144
    elif lead_distance < 160:
      t = (lead_distance - 100) /60.0
      return ( (35 * t**2) + (10 * (1 - t)**2) + (28 * t *(1 - t))) * 0.9144
    else:
      return (lead_distance - 125) * 0.9144

  # lead_distance is non linear
  # Must be called every frame and assumes 100hz (frames per second)
  def rough_speed(self, lead_distance):
    #If we got an updated lead distance calculate the closing rate
    if self.prev_lead_distance != lead_distance:
      #delta distance is negative when approaching
      delta_distance = self.honda_bosch_to_meters(lead_distance) - self.honda_bosch_to_meters(self.prev_lead_distance)
      #delta_speed is distance / time (seconds when called at 100hz)
      delta_speed = delta_distance / self.lead_distance_counter
      #set the rough lead speed by feathering in the updated values
      self.rough_lead_speed = 0.5 * delta_speed + 0.5 * self.rough_lead_speed
      #reset lead distance counter
      self.lead_distance_counter = 0.0
      #update previous lead distance
      self.prev_lead_distance = lead_distance
    #If it has been a while since the last lead distance update, assume delta speed is zero
    elif self.lead_distance_counter >= 2.0:
      #reduce the lead speed to zero
      self.rough_lead_speed = 0
      #set distance counter to above zero to avoid high initial values
      self.lead_distance_counter = 1.0
    #increase counter by 0.01 (1/100 of a second)
    self.lead_distance_counter += 0.01
    print("{0} m/s".format(self.rough_lead_speed))

  # relative distance is in ft from output
  # v_ego is current car speed
  # stopped is a CS telling whether vehicle is in stopped state or not
  def get_TR(self, lead_distance, v_ego, stopped):
    # Radar picks up at about ~230ft
    # lead distance at 1 keeps about ~135ft-151ft
    # Testing out a bunch of random numbers..
    # If car and lead car is moving set to 1 always and let different speed rules take over..
    if (v_ego >= 1) and (self.rough_lead_speed >= 0.1) and (lead_distance <= 230):
      self.desiredTR = 1
    # if lead car is positive relative speed set to 1 to catch up
    elif (self.rough_lead_speed > 0.1):
      self.desiredTR = 1
      # car is slowing down?
      if (self.rough_lead_speed <= 0.1):
        self.desiredTR = 2
      elif (self.rough_lead_speed < -1):
        self.desiredTR = 3
      elif (self.rough_lead_speed < -6):
        self.desiredTR = 4

    # No lead car found
    if lead_distance == 255:
      self.desiredTR = 1
    # Reset to 1 if car is stopped in front of car
    if stopped:
      self.desiredTR = 1
      
    return self.desiredTR

  def update(self, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.v_ego, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., 1./100)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes and CS.lkMode:
      hud_lanes = 1
    else:
      hud_lanes = 0

    # Always detect lead car on HUD even without ACC engaged
    if hud_show_car:
      hud_car = 2
    else:
      hud_car = 1

    #print("{0} {1} {2}".format(chime, alert_id, hud_alert))
    fcw_display, steer_required, acc_alert, ldw_display = process_hud_alert(hud_alert, CS.v_ego)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, fcw_display, acc_alert, steer_required, CS.lkMode, ldw_display)

    # **** process the car messages ****

    # *** compute control surfaces ***

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * P.BRAKE_MAX, 0, P.BRAKE_MAX - 1))
    apply_steer = -actuators.steer * P.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_driver, P)
    self.apply_steer_last = apply_steer

    lkas_active = enabled and not CS.steer_not_allowed and CS.lkMode

    # Send CAN commands.
    can_sends = []

    # Send steering command.
    idx = frame % 4
    can_sends.extend(hondacan.create_steering_control(self.packer, apply_steer,
      lkas_active, CS.CP.carFingerprint, idx, CS.CP.isPandaBlack))

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame//10) % 4
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, CS.CP.carFingerprint, CS.is_metric, idx, CS.CP.isPandaBlack))

    
    if CS.CP.carFingerprint in (CAR.INSIGHT, CAR.ACCORD):
      self.rough_speed(CS.leadDistance)
      if kegman.conf['simpledd'] == True:
        #Get the desiredTR before using it.
        self.get_TR(CS.leadDistance, CS.v_ego, CS.stopped)
        # update to CS so we can push it to ui through cereal
        CS.desiredTR = self.desiredTR
        if frame % 13 < 2 and CS.hud_distance != (self.desiredTR % 4):
          # press distance bar button
          can_sends.append(hondacan.spam_buttons_command(self.packer, 0, CruiseSettings.LEAD_DISTANCE, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
          # always set cruise setting to 0 after button press
          if frame % 25 < 5:
            can_sends.append(hondacan.spam_buttons_command(self.packer, 0, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))

    if CS.CP.radarOffCan:
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      if CS.stopped:
        if CS.CP.carFingerprint in (CAR.INSIGHT):
          if CS.leadDistance > (self.prev_lead_distance + 10.0) or self.rough_lead_speed > 1:
            can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
        else:
          can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      else:
        self.prev_lead_distance = CS.leadDistance
    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = frame // 2
        ts = frame * DT_CTRL
        pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)
        can_sends.append(hondacan.create_brake_command(self.packer, apply_brake, pump_on,
          pcm_override, pcm_cancel_cmd, hud.fcw, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
        self.apply_brake_last = apply_brake

        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          can_sends.append(create_gas_command(self.packer, apply_gas, idx))

    return can_sends
