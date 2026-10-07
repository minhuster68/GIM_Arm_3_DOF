"""Closed R/A paths with analytic joint velocity and acceleration."""

import numpy as np
from scipy.interpolate import CubicSpline

from gim_control import sweep_trajectory as sweep
from gim_control.gim_arm_kinematics import IKResult


def letter_strokes(shape, width=0.28, height=0.20):
    """Return Bezier control points in front-view (x,z), relative to start.

    R starts at the right end of its middle bar; A starts at its right foot.
    Retracing existing strokes closes each letter without adding a new line.
    """
    if shape == "r":
        start, middle = (0.0, 0.0), (-width, 0.0)
        top, bottom = (-width, height / 2), (-width, -height / 2)
        foot = (0.0, -height / 2)
        return [
            [start, (0.0, height / 2), (-0.35 * width, height / 2), top],
            [top, bottom], [bottom, middle], [middle, start],
            [start, middle], [middle, foot], [foot, middle], [middle, start],
        ]
    if shape == "a":
        start, apex, left = (0.0, 0.0), (-width / 2, height), (-width, 0.0)
        bar_left = (-0.75 * width, height / 2)
        bar_right = (-0.25 * width, height / 2)
        return [[start, apex], [apex, left], [left, bar_left],
                [bar_left, bar_right], [bar_right, start]]
    raise ValueError("letter shape phải là 'r' hoặc 'a'")


def stroke_positions(control, anchor, parameters):
    """Project an x/z Bezier stroke onto the reachable shoulder shell."""
    control = np.asarray(control, dtype=float)
    t = np.asarray(parameters, dtype=float)
    if len(control) == 2:
        offsets = (1 - t[..., None]) * control[0] + t[..., None] * control[1]
    else:
        offsets = ((1 - t[..., None])**3 * control[0]
                   + 3 * (1 - t[..., None])**2 * t[..., None] * control[1]
                   + 3 * (1 - t[..., None]) * t[..., None]**2 * control[2]
                   + t[..., None]**3 * control[3])
    anchor = np.asarray(anchor, dtype=float)
    pivot = np.asarray(sweep.SHOULDER_PIVOT)
    radius = np.linalg.norm(anchor - pivot)
    x, z = anchor[0] + offsets[..., 0], anchor[2] + offsets[..., 1]
    forward_squared = radius**2 - (x - pivot[0])**2 - (z - pivot[2])**2
    if np.any(forward_squared <= 0.0):
        raise ValueError("Chữ vượt ra ngoài vùng vươn tay quanh vai")
    y = pivot[1] + np.sqrt(forward_squared)
    return np.stack([x, y, z], axis=-1)


class LetterTrajectory:
    """Spline IK per stroke, timed with quintic minimum-jerk segments.

    q(t)=Q(h(t)), qd=Q'(h)*hd, qdd=Q''(h)*hd²+Q'(h)*hdd.
    Each stroke joins the next with zero velocity and acceleration (C2).
    """

    def __init__(self, kin, shape, q_start, duration, samples_per_stroke=41):
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration phải hữu hạn và > 0")
        if samples_per_stroke < 4:
            raise ValueError("samples_per_stroke phải >= 4")
        self.duration = self.period = float(duration)
        self.shape = shape
        self.anchor = sweep.build_positions()[0]
        parameters = np.linspace(0.0, 1.0, samples_per_stroke)
        controls = letter_strokes(shape)
        current = np.asarray(q_start, dtype=float).copy()
        self.positions, self.results, self.splines = [], [], []
        lengths = []
        for index, control in enumerate(controls):
            positions = stroke_positions(control, self.anchor, parameters)
            stroke_q = [current.copy()]
            first = IKResult(current.copy(), True, 0, float(np.linalg.norm(
                kin.fk_position(current) - positions[0])))
            results = [first]
            for position in positions[1:]:
                result = kin.ik_position(position, q_init=current, eps=1e-10)
                if not result.converged:
                    raise RuntimeError(f"IK chữ {shape.upper()} không hội tụ")
                current = result.q
                results.append(result)
                stroke_q.append(current.copy())
            if index == len(controls) - 1:
                # Exactly the same joint endpoint as the existing circle.
                current = np.asarray(q_start, dtype=float).copy()
                stroke_q[-1] = current
                results[-1] = IKResult(current, True, 0, float(np.linalg.norm(
                    kin.fk_position(current) - positions[-1])))
            self.splines.append(CubicSpline(parameters, stroke_q, axis=0))
            self.positions.extend(positions)
            self.results.extend(results)
            lengths.append(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
        durations = self.duration * np.asarray(lengths) / np.sum(lengths)
        self.boundaries = np.r_[0.0, np.cumsum(durations)]
        self.boundaries[-1] = self.duration
        self.positions = np.asarray(self.positions)

    def at(self, t):
        """Return q, qd, qdd for a scalar or an array of times."""
        times = np.asarray(t, dtype=float)
        clipped = np.clip(times.ravel(), 0.0, self.duration)
        indices = np.minimum(
            np.searchsorted(self.boundaries[1:], clipped, side="right"),
            len(self.splines) - 1)
        outputs = [np.empty((clipped.size, 3)) for _ in range(3)]
        for index in np.unique(indices):
            mask = indices == index
            duration = self.boundaries[index + 1] - self.boundaries[index]
            u = np.clip((clipped[mask] - self.boundaries[index]) / duration, 0, 1)
            h = 10*u**3 - 15*u**4 + 6*u**5
            hd = (30*u**2 - 60*u**3 + 30*u**4) / duration
            hdd = (60*u - 180*u**2 + 120*u**3) / duration**2
            spline = self.splines[index]
            outputs[0][mask] = spline(h)
            outputs[1][mask] = spline(h, 1) * hd[:, None]
            outputs[2][mask] = (spline(h, 2) * hd[:, None]**2
                                + spline(h, 1) * hdd[:, None])
        return tuple(value.reshape(times.shape + (3,)) for value in outputs)
