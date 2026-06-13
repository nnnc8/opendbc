import unittest
from unittest.mock import Mock, patch
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car import structs, Bus
from opendbc.car.toyota.values import CAR

class TestToyotaQuietCabin(unittest.TestCase):
  @patch("opendbc.car.toyota.carcontroller.Params")
  @patch("opendbc.car.toyota.toyotacan.create_ui_command")
  def test_quiet_cabin_behavior(self, mock_create_ui_command, mock_params_cls):
    # Set up basic CarParams
    CP = structs.CarParams.new_message()
    CP.carFingerprint = CAR.TOYOTA_COROLLA_TSS2
    CP.openpilotLongitudinalControl = False
    CP.flags = 0

    dbc_names = {Bus.pt: "toyota_nodsu_pt_generated"}

    # Mock parameters instance
    mock_params_inst = Mock()
    mock_params_cls.return_value = mock_params_inst

    # Setup standard Mock CC and CS
    CC = Mock()
    CC.actuators.torque = 0.0
    CC.actuators.steeringAngleDeg = 0.0
    CC.actuators.accel = 0.0
    CC.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
    CC.hudControl.leftLaneVisible = False
    CC.hudControl.rightLaneVisible = False
    CC.hudControl.leftLaneDepart = False
    CC.hudControl.rightLaneDepart = False
    CC.hudControl.leadVisible = False
    CC.hudControl.leadDistanceBars = 0
    CC.cruiseControl.cancel = True # Crucial trigger for double beep (chime)
    CC.latActive = True
    CC.orientationNED = [0.0, 0.0, 0.0]

    CS = Mock()
    CS.out.steeringTorque = 0
    CS.out.steeringTorqueEps = 0
    CS.out.steeringRateDeg = 0.0
    CS.out.steeringAngleOffsetDeg = 0.0
    CS.out.steeringAngleDeg = 0.0
    CS.out.vEgoRaw = 10.0
    CS.out.vEgo = 10.0
    CS.out.aEgo = 0.0
    CS.out.brakeholdGovernor = False
    CS.out.standstill = False
    CS.out.cruiseState.enabled = True
    CS.out.cruiseState.available = True
    CS.pcm_follow_distance = 3
    CS.gvc = 0.0
    CS.secoc_synchronization = {'TRIP_CNT': 0, 'RESET_CNT': 0, 'AUTHENTICATOR': 0}
    CS.lkas_hud = {}
    CS.acc_type = 1

    # --- Case 1: AegisQuietCabin is False ---
    mock_params_inst.get_bool.return_value = False
    
    controller = CarController(dbc_names, CP)
    # Trigger UI send ASAP with cancel cmd
    controller.update(CC, CS, 0)

    # Verify create_ui_command was called with chime = True (pcm_cancel_cmd is True)
    mock_create_ui_command.assert_called()
    args, kwargs = mock_create_ui_command.call_args
    # create_ui_command signature: (packer, steer, chime, ...)
    chime_passed = args[2]
    self.assertTrue(chime_passed)

    # Reset mock for Case 2
    mock_create_ui_command.reset_mock()

    # --- Case 2: AegisQuietCabin is True ---
    mock_params_inst.get_bool.return_value = True
    
    controller = CarController(dbc_names, CP)
    # Trigger UI send ASAP with cancel cmd
    controller.update(CC, CS, 0)

    # Verify create_ui_command was called with chime = 0 (suppressed!)
    mock_create_ui_command.assert_called()
    args, kwargs = mock_create_ui_command.call_args
    chime_passed = args[2]
    self.assertEqual(chime_passed, 0)


if __name__ == "__main__":
  unittest.main()
