#!/usr/bin/env python3
"""Export path references with joint/Cartesian position, velocity, acceleration."""

import argparse
import csv
import logging
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin

from gim_control import sweep_trajectory as sweep
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.reference_trajectory import load_trajectory


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "results/trajectory_shapes")
    parser.add_argument("--shapes", nargs="+", choices=("circle", "r", "a", "square"),
                        default=("circle", "r", "a", "square"))
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    urdf = ROOT / "src/gim_arm_description/urdf/gim_arm.urdf"
    kin = GimArmKinematics(str(urdf), tool_offset_xyz=sweep.TOOL_OFFSET)
    logger = logging.getLogger("trajectory-preview")
    count = len(args.shapes)
    figure = plt.figure(figsize=(max(6.5, 4.5*count), 8), constrained_layout=True)
    for index, shape in enumerate(args.shapes):
        profile = load_trajectory(str(urdf), "", logger, shape)
        times = np.linspace(0.0, profile.duration, 2701)
        q, qd, qdd = profile.at(times)
        xyz = np.asarray([kin.fk_position(value) for value in q])
        velocity, acceleration = [], []
        for position, speed, accel in zip(q, qd, qdd):
            pin.forwardKinematics(kin.model, kin.data, position, speed, accel)
            pin.updateFramePlacements(kin.model, kin.data)
            velocity.append(pin.getFrameVelocity(
                kin.model, kin.data, kin.ee_frame_id, pin.LOCAL_WORLD_ALIGNED).linear.copy())
            acceleration.append(pin.getFrameClassicalAcceleration(
                kin.model, kin.data, kin.ee_frame_id, pin.LOCAL_WORLD_ALIGNED).linear.copy())
        with (output / f"{shape}_reference.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(["t", "x", "y", "z", "q1", "q2", "q3",
                             "qd1", "qd2", "qd3", "qdd1", "qdd2", "qdd3",
                             "vx", "vy", "vz", "ax", "ay", "az"])
            writer.writerows(np.column_stack([times, xyz, q, qd, qdd, velocity, acceleration]))
        front = figure.add_subplot(2, count, index + 1)
        front.plot(xyz[:, 0], xyz[:, 2], linewidth=2)
        front.scatter(xyz[0, 0], xyz[0, 2], marker="*", s=100, c="tab:red",
                      label="Start = end")
        front.set(title=f"{shape.upper()} — front view, {profile.duration:g} s",
                  xlabel="x (m)", ylabel="z (m)")
        front.set_aspect("equal", adjustable="box")
        front.grid(alpha=0.3)
        front.legend(fontsize=8)
        spatial = figure.add_subplot(2, count, index + count + 1, projection="3d")
        spatial.plot(*xyz.T)
        spatial.scatter(*xyz[0], marker="*", s=80, c="tab:red")
        spatial.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        print(f"{shape}: {profile.duration:g}s; max |qd|={np.abs(qd).max(axis=0)}; "
              f"max |qdd|={np.abs(qdd).max(axis=0)}")
        joints, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                                    constrained_layout=True)
        for axis, values, label in zip(axes, (q, qd, qdd),
                                       ('q (rad)', 'qd (rad/s)', 'qdd (rad/s²)')):
            axis.plot(times, values, label=kin.joint_names)
            axis.set_ylabel(label)
            axis.grid(alpha=.3)
        axes[0].legend()
        axes[-1].set_xlabel('t (s)')
        joints.suptitle(f'{shape.upper()} — analytic joint reference')
        joints.savefig(output / f'{shape}_joint_reference.png', dpi=160)
        plt.close(joints)
    figure.savefig(output / "reference_paths.png", dpi=180)
    figure.savefig(output / "reference_paths.svg")
    plt.close(figure)
    print(output / "reference_paths.png")


if __name__ == "__main__":
    main()
