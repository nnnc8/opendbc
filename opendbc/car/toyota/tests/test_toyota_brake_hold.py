import unittest
from unittest.mock import Mock, patch
from collections import defaultdict
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car import structs, Bus
from opendbc.car.toyota.values import CAR, ToyotaFlags

class MockParser:
  def __init__(self):
    self.vl = defaultdict(lambda: defaultdict(lambda: 0.0))

class MockCanParsers:
  def __init__(self):
    self.pt_parser = MockParser()
    self.cam_parser = MockParser()
  def __getitem__(self, bus):
    if bus == Bus.cam:
      return self.cam_parser
    return self.pt_parser

class TestToyotaBrakeHold(unittest.TestCase):
  def setUp(self):
    self.CP = structs.CarParams.new_message()
    self.CP.mass = 1500.0
    self.CP.wheelSpeedFactor = 1.0
    self.CP.carFingerprint = CAR.TOYOTA_COROLLA_TSS2
    self.CP.flags = ToyotaFlags.AUTO_BRAKE_HOLD.value | ToyotaFlags.HYBRID.value
    self.CS = CarState(self.CP)
    self.CS.AutomaticBrakeHold = True
    self.can_parsers = MockCanParsers()
    
    # Mock parse_gear_shifter to return drive by default
    self.CS.parse_gear_shifter = lambda gear: structs.CarState.GearShifter.drive

    # Default satisfied inputs
    self.set_satisfied_inputs()

  def set_satisfied_inputs(self):
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] = 0.0
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] = 0.0
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"] = 0.0
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"] = 0.0
    self.can_parsers.pt_parser.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] = 1.0
    self.can_parsers.pt_parser.vl["PCM_CRUISE"]["GAS_RELEASED"] = 1.0
    self.can_parsers.pt_parser.vl["PCM_CRUISE_2"]["MAIN_ON"] = 1.0
    self.can_parsers.pt_parser.vl["PCM_CRUISE"]["CRUISE_ACTIVE"] = 0.0
    self.can_parsers.pt_parser.vl["GEAR_PACKET"]["GEAR"] = 0.0

  def test_flat_road_hold(self):
    # Scenario 1: Flat road - standstill, brake pressed, stopped for 1.0s (100 frames) -> latched
    self.CS.brakehold_state = "idle"
    
    # First frame satisfies condition -> arming
    out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "arming")
    self.assertFalse(out.brakeholdGovernor)

    # Keep updating for 99 more frames (total 100 frames) -> still arming
    for _ in range(99):
      out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "arming")
    self.assertFalse(out.brakeholdGovernor)

    # 101st frame -> latched!
    out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "latched")
    self.assertTrue(out.brakeholdGovernor)

  def test_downhill_hold(self):
    # Scenario 2: Downhill (ASLP pitch is negative, e.g., -5.0) -> should still enter hold
    self.CS.brakehold_state = "idle"
    self.can_parsers.pt_parser.vl["VSC1S07"]["ASLP"] = -5.0

    # Run 101 frames to trigger hold
    for _ in range(101):
      out = self.CS.update(self.can_parsers)

    # Should enter latched regardless of downhill slope
    self.assertEqual(self.CS.brakehold_state, "latched")
    self.assertTrue(out.brakeholdGovernor)

  def test_creeping_does_not_hold(self):
    # Scenario 3: Creeping/short stop (< 1.0s) -> conditions lost before 1.0s -> reset to idle
    self.CS.brakehold_state = "idle"

    # Satisfied for 50 frames
    for _ in range(50):
      self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "arming")

    # Standstill lost (car starts creeping)
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] = 5.0
    self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "idle")

  def test_standstill_jitter_ignored_when_latched(self):
    # Scenario 4: Once latched, brief standstill jitter or slope change does not drop out
    self.CS.brakehold_state = "latched"

    # Standstill jitters to False for a brief moment (e.g. 10 frames)
    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] = 5.0
    for _ in range(10):
      out = self.CS.update(self.can_parsers)
    
    # Should still be latched!
    self.assertEqual(self.CS.brakehold_state, "latched")
    self.assertTrue(out.brakeholdGovernor)

  def test_gas_release(self):
    # Scenario 5: Gas pressed -> instant release
    self.CS.brakehold_state = "latched"

    self.can_parsers.pt_parser.vl["PCM_CRUISE"]["GAS_RELEASED"] = 0.0 # Gas pressed

    out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "idle")
    self.assertFalse(out.brakeholdGovernor)

  def test_gear_rp_release(self):
    # Scenario 6: Gear shifted to reverse/park -> instant release
    for gear in (structs.CarState.GearShifter.reverse, structs.CarState.GearShifter.park):
      self.CS.brakehold_state = "latched"
      self.CS.parse_gear_shifter = lambda g: gear

      out = self.CS.update(self.can_parsers)
      self.assertEqual(self.CS.brakehold_state, "idle")
      self.assertFalse(out.brakeholdGovernor)

  def test_safe_release_on_movement(self):
    # Scenario 7: Vehicle moves (> 0.3s) without gas or cruise enabled -> safe release
    self.CS.brakehold_state = "latched"

    self.can_parsers.pt_parser.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] = 5.0 # moving

    # 29 frames of no standstill -> still latched
    for _ in range(29):
      out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "latched")

    # 30th frame -> safe release to idle!
    out = self.CS.update(self.can_parsers)
    self.assertEqual(self.CS.brakehold_state, "idle")
    self.assertFalse(out.brakeholdGovernor)

  def test_controller_governor_behavior(self):
    # Scenario 8: Controller governor checks
    self.CP.openpilotLongitudinalControl = True
    dbc_names = {Bus.pt: "toyota_nodsu_pt_generated"}
    CC = Mock()
    CC.actuators.accel = 0.5
    CC.actuators.torque = 0.0
    CC.cruiseControl.cancel = False
    CC.latActive = False
    CC.longActive = True
    CC.orientationNED = [0.0, 0.0, 0.0]
    CC.hudControl = Mock()
    CC.hudControl.leadDistanceBars = 1
    
    CS = Mock()
    CS.out.standstill = True
    CS.out.vEgo = 0.0
    CS.out.aEgo = 0.0
    CS.out.cruiseState.enabled = False
    CS.out.cruiseState.available = True
    CS.out.steeringTorque = 0
    CS.out.steeringTorqueEps = 0
    CS.out.steeringRateDeg = 0.0
    CS.out.steeringAngleOffsetDeg = 0.0
    CS.out.steeringAngleDeg = 0.0
    CS.acc_type = 1
    CS.secoc_synchronization = None
    CS.lkas_hud = {}
    CS.gvc = 0.0
    
    # When governor is True -> pcm_accel_cmd should be overridden to 0.0, standstill_req is False
    CS.out.brakeholdGovernor = True
    
    controller = CarController(dbc_names, self.CP)
    with patch("opendbc.car.toyota.toyotacan.create_accel_command") as mock_create_accel:
      controller.update(CC, CS, 0)
      mock_create_accel.assert_called()
      args, kwargs = mock_create_accel.call_args
      pcm_accel_cmd_passed = args[1]
      standstill_req_passed = args[5]
      
      self.assertEqual(pcm_accel_cmd_passed, 0.0)
      self.assertFalse(standstill_req_passed)

if __name__ == "__main__":
  unittest.main()
