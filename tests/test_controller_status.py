import unittest

from t800_ros_api.controller_status import parse_controller_status


class ControllerStatusTests(unittest.TestCase):
    def test_armed_status(self):
        raw = (
            'state=ARMED|reason=ARMED|event=TRIPLE_Y|last_disarm=STARTUP_DISARMED|'
            'motion_allowed=1|gamepad_connected=1|sticks_neutral=1|'
            'RX Twist: vx=0.123 vy=-0.300 wz=0.000|twist_age=0.041s|'
            'TX /motion/body_vel_cmd: vx=0.123 vy=-0.300 wz=0.000'
        )
        result = parse_controller_status(raw)
        self.assertEqual(result['state'], 'ARMED')
        self.assertEqual(result['reason'], 'ARMED')
        self.assertTrue(result['motion_allowed'])
        self.assertTrue(result['gamepad_connected'])
        self.assertEqual(result['rx_twist']['vy'], -0.3)
        self.assertEqual(result['tx_topic'], '/motion/body_vel_cmd')
        self.assertEqual(result['tx_twist']['vx'], 0.123)
        self.assertAlmostEqual(result['twist_age_s'], 0.041)

    def test_disarmed_no_twist(self):
        raw = (
            'state=DISARMED|reason=GAMEPAD_TIMEOUT|motion_allowed=0|'
            'gamepad_connected=0|sticks_neutral=0|RX Twist: NONE|'
            'twist_age=N/A|TX /motion/body_vel_cmd: vx=0.000 vy=0.000 wz=0.000'
        )
        result = parse_controller_status(raw)
        self.assertFalse(result['motion_allowed'])
        self.assertIsNone(result['rx_twist'])
        self.assertIsNone(result['twist_age_s'])
        self.assertEqual(result['tx_twist']['wz'], 0.0)

    def test_invalid_values_are_not_exposed_as_nan(self):
        result = parse_controller_status('RX Twist: vx=nan vy=0.0 wz=inf|twist_age=nans')
        self.assertIsNone(result['rx_twist'])
        self.assertIsNone(result['twist_age_s'])


if __name__ == '__main__':
    unittest.main()
