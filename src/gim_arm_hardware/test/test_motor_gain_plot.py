"""Check a rendered trial uses degrees and the correct three-joint signals."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'tune_motor_gains.py'
SPEC = importlib.util.spec_from_file_location('motor_gain_tuner', SCRIPT)
tuner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tuner)


class TestMotorGainPlot(unittest.TestCase):
    def test_png_shows_selected_joint_in_degrees_and_other_joints_hold_error(self):
        rows = [
            [0.0, *map(math.radians, [10, 20, 0]), *map(math.radians, [10, 20, 0])],
            [6.0, *map(math.radians, [10.1, 19.8, 29]), *map(math.radians, [10, 20, 30])],
            [18.0, *map(math.radians, [10, 20, 0.5]), *map(math.radians, [10, 20, 0])],
        ]
        figures = []
        subplots = plt.subplots

        def capture(*args, **kwargs):
            figure, axes = subplots(*args, **kwargs)
            figures.append((figure, axes))
            return figure, axes

        with tempfile.TemporaryDirectory() as temp:
            csv_path = Path(temp) / 'trial.csv'
            with patch.object(plt, 'subplots', side_effect=capture):
                with patch.object(plt, 'show') as show:
                    path = tuner.plot_trial(rows, 2, dict(kpp=20.0, kvp=0.16, kvi=0.0),
                                            csv_path, 6, 3, show=False)
            self.assertEqual(path, csv_path.with_suffix('.png'))
            self.assertGreater(path.stat().st_size, 1000)
            show.assert_not_called()
        figure, axes = figures[0]
        for measured, expected in zip(axes[0].lines[0].get_ydata(), [0.0, 30.0, 0.0]):
            self.assertAlmostEqual(measured, expected)
        for measured, expected in zip(axes[0].lines[1].get_ydata(), [0.0, 29.0, 0.5]):
            self.assertAlmostEqual(measured, expected)
        for measured, expected in zip(axes[1].lines[0].get_ydata(), [0.0, -1.0, 0.5]):
            self.assertAlmostEqual(measured, expected)
        self.assertAlmostEqual(axes[2].lines[0].get_ydata()[1], 0.1)
        self.assertAlmostEqual(axes[2].lines[1].get_ydata()[1], -0.2)
        self.assertIn('kpp=20', figure._suptitle.get_text())
        self.assertNotIn(figure.number, plt.get_fignums())

    def test_empty_feedback_does_not_create_a_misleading_plot(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'empty.csv'
            result = tuner.plot_trial([], 2, dict(kpp=None, kvp=None, kvi=None), path, 6, 3)
            self.assertIsNone(result)
            self.assertFalse(path.with_suffix('.png').exists())

    def test_gui_plot_opens_and_closes_after_saving(self):
        # Load the actual Agg canvas, then mock only the desktop display call.
        plt.switch_backend('Agg')
        rows = [[0.0, 0, 0, 0, 0, 0, 0], [1.0, 0, 0, 0.1, 0, 0, 0.1]]
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(matplotlib, 'get_backend', return_value='TkAgg'):
                with patch.object(plt, 'show') as show:
                    path = tuner.plot_trial(rows, 2, dict(kpp=20, kvp=0.16, kvi=0),
                                            Path(temp) / 'gui.csv', 6, 3)
                    self.assertTrue(path.is_file())
                    show.assert_called_once_with(block=True)


if __name__ == '__main__':
    unittest.main()
