#!/usr/bin/env python3
"""Xuất quỹ đạo hoàn chỉnh sang MATLAB.

Quỹ đạo gồm ba đoạn liên tục về q, qdot và qdd:
  1. tư thế buông thõng q=[0,0,0] -> điểm đầu quỹ đạo;
  2. chạy đúng một vòng quỹ đạo quét;
  3. từ điểm khép vòng -> trở lại q=[0,0,0].
"""

import argparse
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.io import savemat

from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control import sweep_trajectory


JOINT_NAMES = ("base_joint", "shoulder_joint", "elbow_joint")
HANGING_Q = np.zeros(3, dtype=float)


def minimum_jerk_segment(q0, q1, duration, sample_dt):
    """Nội suy bậc 5 với qdot=qddot=0 ở cả hai đầu."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if duration <= 0.0 or sample_dt <= 0.0:
        raise ValueError("duration và sample_dt phải > 0")

    count = max(1, int(np.ceil(duration / sample_dt)))
    t = np.linspace(0.0, duration, count + 1)
    u = t / duration
    dq = q1 - q0

    h = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    hd = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
    hdd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / duration**2

    q = q0 + h[:, None] * dq
    qd = hd[:, None] * dq
    qdd = hdd[:, None] * dq
    return t, q, qd, qdd


def build_complete_trajectory(kin, sample_dt=0.01):
    """Sinh trajectory: buông thõng -> quét một vòng -> buông thõng."""
    positions, results = sweep_trajectory.solve(kin)
    ok, lines = sweep_trajectory.safety_report(kin, positions, results)
    if not ok:
        raise RuntimeError("Quỹ đạo không đạt safety_report:\n" + "\n".join(lines))

    q_way = np.asarray([result.q for result in results], dtype=float)
    sweep = sweep_trajectory.SmoothJointProfile(q_way, sweep_trajectory.DT)

    t_approach, q_approach, qd_approach, qdd_approach = minimum_jerk_segment(
        HANGING_Q, q_way[0], sweep_trajectory.TRANSITION_TIME, sample_dt)
    t_sweep, q_sweep, qd_sweep, qdd_sweep = sweep.sample(sample_dt)
    t_return, q_return, qd_return, qdd_return = minimum_jerk_segment(
        q_sweep[-1], HANGING_Q, sweep_trajectory.RETURN_TIME, sample_dt)

    # Bỏ mẫu đầu của đoạn 2 và 3 vì trùng với mẫu cuối đoạn trước.
    t_sweep = t_sweep[1:] + t_approach[-1]
    t_return = t_return[1:] + t_sweep[-1]
    t = np.concatenate([t_approach, t_sweep, t_return])
    q = np.vstack([q_approach, q_sweep[1:], q_return[1:]])
    qd = np.vstack([qd_approach, qd_sweep[1:], qd_return[1:]])
    qdd = np.vstack([qdd_approach, qdd_sweep[1:], qdd_return[1:]])

    segment = np.concatenate([
        np.zeros(len(t_approach), dtype=np.uint8),
        np.ones(len(t_sweep), dtype=np.uint8),
        np.full(len(t_return), 2, dtype=np.uint8),
    ])
    return t, q, qd, qdd, segment, np.asarray(positions), lines


def default_urdf():
    return os.path.join(
        get_package_share_directory("gim_arm_description"),
        "urdf", "gim_arm.urdf")


def main():
    parser = argparse.ArgumentParser(
        description="Xuất trajectory.mat: buông thõng -> quét 1 vòng -> buông thõng.")
    parser.add_argument("--output", default="trajectory.mat", help="file .mat đầu ra")
    parser.add_argument("--sample-dt", type=float, default=0.01,
                        help="chu kỳ lấy mẫu, giây (mặc định 0.01 = 100 Hz)")
    parser.add_argument("--urdf", default=default_urdf(), help="đường dẫn URDF")
    args = parser.parse_args()

    kin = GimArmKinematics(args.urdf, tool_offset_xyz=sweep_trajectory.TOOL_OFFSET)
    if tuple(kin.joint_names) != JOINT_NAMES:
        raise RuntimeError(
            f"Thứ tự khớp trong URDF là {kin.joint_names}, mong đợi {list(JOINT_NAMES)}")

    t, q, qd, qdd, segment, cartesian_waypoints, report = (
        build_complete_trajectory(kin, args.sample_dt))

    savemat(args.output, {
        "t": t[:, None],
        "q": q,
        "qd": qd,
        "qdd": qdd,
        "segment": segment[:, None],
        "joint_names": np.asarray(JOINT_NAMES, dtype=object),
        "initial_q": HANGING_Q[None, :],
        "cartesian_waypoints": cartesian_waypoints,
        "sample_dt": float(args.sample_dt),
        "transition_time": float(sweep_trajectory.TRANSITION_TIME),
        "sweep_time": float(len(cartesian_waypoints) * sweep_trajectory.DT),
        "return_time": float(sweep_trajectory.RETURN_TIME),
    })

    print("\n".join(report))
    print(f"\nĐã lưu: {os.path.abspath(args.output)}")
    print(f"Số mẫu: {len(t)} | thời gian: {t[-1]:.2f}s | dt: {args.sample_dt:g}s")
    print("Các biến MATLAB: t, q, qd, qdd, segment, joint_names, initial_q, "
          "cartesian_waypoints")
    print("segment: 0=đi tới quỹ đạo, 1=quét một vòng, 2=quay về buông thõng")


if __name__ == "__main__":
    main()
