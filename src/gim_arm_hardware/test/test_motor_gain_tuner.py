"""Verify motor gain frames using a fake transport, never real CAN."""

import contextlib
import importlib.util
import io
import math
from pathlib import Path
import struct
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'tune_motor_gains.py'
SPEC = importlib.util.spec_from_file_location('motor_gain_tuner', SCRIPT)
tuner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tuner)


class FakeSender:
    def __init__(self):
        self.frames = []
        self.fail_command = None

    def send(self, node, command, payload):
        if command == self.fail_command:
            raise OSError('mock send failure')
        self.frames.append(tuner.make_frame(node, command, payload))


class TestMotorGainTuner(unittest.TestCase):
    def setUp(self):
        self.sender = FakeSender()
        self.terminal = tuner.GainTerminal(self.sender, 2)
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()

    def tearDown(self):
        self.output.__exit__(None, None, None)

    def test_set_encodes_driver_gains_and_node_id_with_exact_dlc(self):
        self.terminal.execute('set 20 0.16 0')
        position, velocity = [tuner.CAN_FRAME.unpack(frame) for frame in self.sender.frames]
        self.assertEqual(position[:2], (0x05A, 4))
        self.assertEqual(position[2], struct.pack('<f', 20) + b'\x00' * 4)
        self.assertEqual(velocity[:2], (0x05B, 8))
        self.assertEqual(velocity[2], struct.pack('<ff', 0.16, 0))

    def test_initial_individual_velocity_edit_does_not_overwrite_unknown_gain(self):
        for command in ('kvp 0.16', 'kvi 0'):
            with self.assertRaisesRegex(ValueError, 'Chưa biết'):
                self.terminal.execute(command)
        self.assertEqual(self.sender.frames, [])

    def test_individual_updates_preserve_partner_and_per_node_history(self):
        self.terminal.execute('vel 0.16 0.32')
        self.terminal.execute('kvi 0')
        self.terminal.execute('kvp 0.2')
        self.assertEqual(self.sender.frames[-1][8:], struct.pack('<ff', 0.2, 0))
        self.terminal.execute('joint shoulder')
        with self.assertRaises(ValueError):
            self.terminal.execute('kvi 0')
        self.terminal.execute('vel 0.3 0.1')
        self.terminal.execute('joint elbow')
        self.terminal.execute('kvi 0.04')
        self.assertEqual(self.sender.frames[-1][8:], struct.pack('<ff', 0.2, 0.04))

    def test_invalid_inputs_send_nothing_even_when_last_gain_is_invalid(self):
        for line in ('set 20 0.16 nan', 'set 20 -1 0', 'set 20 0.16 1e100',
                     'set 20 0.16 1e-100', 'kpp inf', 'node -1', 'node 64',
                     'node 2.5', 'joint wrist', 'vel 0.1', 'set 1 2', 'save extra'):
            with self.subTest(line=line), self.assertRaises(ValueError):
                self.terminal.execute(line)
        self.assertEqual(self.sender.frames, [])
        self.assertEqual(self.terminal.node, 2)

    def test_save_is_explicit_and_exit_sends_nothing(self):
        self.terminal.show()
        self.terminal.execute('help')
        self.assertEqual(self.sender.frames, [])
        self.terminal.execute('save')
        saved = tuner.CAN_FRAME.unpack(self.sender.frames[0])
        self.assertEqual(saved, (0x05F, 0, b'\x00' * 8))
        self.assertFalse(self.terminal.execute('quit'))
        self.assertEqual(len(self.sender.frames), 1)

    def test_send_failure_does_not_claim_failed_gain_and_retains_partial_set(self):
        self.sender.fail_command = tuner.SET_VEL_GAINS
        with self.assertRaises(OSError):
            self.terminal.execute('set 20 0.16 0')
        self.assertEqual(self.terminal.gains, {'kpp': 20.0, 'kvp': None, 'kvi': None})
        self.assertEqual(len(self.sender.frames), 1)

    def test_dry_run_accepts_scripted_terminal_without_opening_a_socket(self):
        from unittest.mock import patch
        with patch.object(tuner, 'SocketCanSender') as socket_sender, patch(
                'builtins.input', side_effect=['set 20 0.16 0', 'kvi 0.04', 'quit']):
            self.assertEqual(tuner.main([
                '--driver-gains', '--dry-run', '--joint', 'elbow']), 0)
        socket_sender.assert_not_called()

    def test_trial_plan_moves_only_selected_joint_and_returns_to_start(self):
        start = [0.1, 0.2, 0.3]
        limits = tuner.load_joint_limits()
        plan = tuner.trial_plan(start, 2, 30, 6, 3, limits)
        self.assertEqual([seconds for seconds, _ in plan], [0, 6, 9, 15, 18])
        self.assertEqual(plan[1][1][:2], start[:2])
        self.assertAlmostEqual(plan[1][1][2], start[2] + math.radians(30))
        self.assertEqual(plan[-1][1], start)
        self.assertEqual(start, [0.1, 0.2, 0.3])
        with self.assertRaises(ValueError):
            tuner.trial_plan(start, 2, 200, 6, 3, limits)
        with self.assertRaisesRegex(ValueError, 'vận tốc'):
            tuner.trial_plan(start, 2, 30, 0.01, 3, limits)

    def test_trial_command_routes_to_ros_without_direct_can_position_frames(self):
        from unittest.mock import Mock
        self.terminal.trial = Mock()
        self.terminal.execute('test 30 6 3')
        self.terminal.trial.run.assert_called_once_with(
            2, 30.0, 6.0, 3.0, {'kpp': None, 'kvp': None, 'kvi': None})
        self.assertEqual(self.sender.frames, [])
        self.terminal.execute('test 5')
        self.assertEqual(self.terminal.trial.run.call_args.args[1:4], (5.0, 6.0, 3.0))

    def test_invalid_trial_never_creates_ros_client_or_sends_can(self):
        from unittest.mock import patch
        with patch.object(tuner, 'RosJointTrial') as trial:
            for line in ('test nan', 'test 30 0', 'test 30 6 -1', 'test 30 inf',
                         'test 30 6 nan', 'test 30 1e-15', 'test 30 1e20', 'test 30 1e300'):
                with self.subTest(line=line), self.assertRaises(ValueError):
                    self.terminal.execute(line)
            self.terminal.execute('node 4')
            with self.assertRaises(ValueError):
                self.terminal.execute('test 30')
            trial.assert_not_called()
        self.assertEqual(self.sender.frames, [])

    def test_dry_run_trial_does_not_open_ros_client(self):
        from unittest.mock import patch
        terminal = tuner.GainTerminal(tuner.DryRunSender(), 2)
        with patch.object(tuner, 'RosJointTrial') as trial:
            terminal.execute('test 30 6 3')
            trial.assert_not_called()


if __name__ == '__main__':
    unittest.main()
