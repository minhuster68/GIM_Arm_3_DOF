"""Quỹ đạo tham chiếu dùng chung cho PID, LQR, MPC và SMC."""

import os

import numpy as np

from gim_control import sweep_trajectory
from gim_control.gim_arm_kinematics import GimArmKinematics


class SmoothSweep:
    """
    Một vòng quét có qdot=qddot=0 ở cả đầu và cuối.

    Dùng đúng ``SmoothJointProfile`` của đường position-controller và file
    xuất MATLAB.  Việc time-scale minimum-jerk là quan trọng: APPROACH kết
    thúc với vận tốc bằng 0, nên TRACK cũng phải bắt đầu bằng 0 để mô-men
    feedforward không nhảy bậc.
    """

    def __init__(self, q_way, dt_way):
        self._profile = sweep_trajectory.SmoothJointProfile(q_way, dt_way)
        self.duration = self._profile.duration
        # Giữ alias này để code phân tích cũ vẫn đọc được độ dài một vòng.
        self.period = self.duration

    def at(self, t):
        return self._profile.at(np.clip(float(t), 0.0, self.duration))


class Quintic:
    """Đoạn nối minimum-jerk, qdot=qddot=0 ở hai đầu."""

    def __init__(self, q0, q1, duration):
        self.q0 = np.asarray(q0, dtype=float)
        self.q1 = np.asarray(q1, dtype=float)
        self.duration = float(duration)

    def at(self, t):
        if self.duration <= 0.0:
            zero = np.zeros_like(self.q1)
            return self.q1.copy(), zero, zero.copy()
        u = np.clip(float(t) / self.duration, 0.0, 1.0)
        delta = self.q1 - self.q0
        h = 10.0*u**3 - 15.0*u**4 + 6.0*u**5
        hd = (30.0*u**2 - 60.0*u**3 + 30.0*u**4) / self.duration
        hdd = (60.0*u - 180.0*u**2 + 120.0*u**3) / self.duration**2
        return self.q0 + h*delta, hd*delta, hdd*delta


class Hold:
    def __init__(self, q):
        self.q = np.asarray(q, dtype=float)

    def at(self, _t):
        zero = np.zeros_like(self.q)
        return self.q.copy(), zero, zero.copy()


class TimedHold(Hold):
    """Tham chiếu đứng yên có thời lượng, dùng cho bài test controller."""

    def __init__(self, q, duration):
        super().__init__(q)
        self.duration = float(duration)
        if not np.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError("duration của TimedHold phải là số hữu hạn > 0")


def solve_waypoints(urdf, cache, logger):
    """Giải và kiểm cùng một quỹ đạo cho mọi thuật toán."""
    modified = os.path.getmtime(urdf)
    if cache and os.path.exists(cache):
        try:
            saved = np.load(cache)
            if (float(saved["mt"]) == modified
                    and int(saved["n"]) == sweep_trajectory.N_POINTS):
                logger.info(f"Dùng waypoint đã cache: {cache}")
                return saved["q"], float(saved["dt"])
        except Exception:
            pass

    kin = GimArmKinematics(
        urdf, end_effector_frame="lower_arm_link",
        tool_offset_xyz=sweep_trajectory.TOOL_OFFSET)
    positions, results = sweep_trajectory.solve(kin)
    ok, lines = sweep_trajectory.safety_report(kin, positions, results)
    for line in lines:
        logger.info(line)
    if not ok:
        raise RuntimeError("Quỹ đạo không đạt safety_report; không tạo lệnh mô-men")

    q_way = np.asarray([result.q for result in results], dtype=float)
    if cache:
        np.savez(cache, q=q_way, dt=float(sweep_trajectory.DT),
                 mt=modified, n=sweep_trajectory.N_POINTS)
    return q_way, float(sweep_trajectory.DT)
