#!/usr/bin/env python3
"""Export circle/R/A references and their front-view and 3D previews."""

import argparse
import csv
import logging
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gim_control import sweep_trajectory as sweep
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.reference_trajectory import load_trajectory


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "results/trajectory_shapes")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    urdf = ROOT / "src/gim_arm_description/urdf/gim_arm.urdf"
    kin = GimArmKinematics(str(urdf), tool_offset_xyz=sweep.TOOL_OFFSET)
    logger = logging.getLogger("trajectory-preview")
    figure = plt.figure(figsize=(13, 8), constrained_layout=True)
    for index, shape in enumerate(("circle", "r", "a")):
        profile = load_trajectory(str(urdf), "", logger, shape)
        times = np.linspace(0.0, profile.duration, 2701)
        q, qd, qdd = profile.at(times)
        xyz = np.asarray([kin.fk_position(value) for value in q])
        with (output / f"{shape}_reference.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(["t", "x", "y", "z", "q1", "q2", "q3",
                             "qd1", "qd2", "qd3", "qdd1", "qdd2", "qdd3"])
            writer.writerows(np.column_stack([times, xyz, q, qd, qdd]))
        front = figure.add_subplot(2, 3, index + 1)
        front.plot(xyz[:, 0], xyz[:, 2], linewidth=2)
        front.scatter(xyz[0, 0], xyz[0, 2], marker="*", s=100, c="tab:red",
                      label="Start = end")
        front.set(title=f"{shape.upper()} — front view, {profile.duration:g} s",
                  xlabel="x (m)", ylabel="z (m)")
        front.set_aspect("equal", adjustable="box")
        front.grid(alpha=0.3)
        front.legend(fontsize=8)
        spatial = figure.add_subplot(2, 3, index + 4, projection="3d")
        spatial.plot(*xyz.T)
        spatial.scatter(*xyz[0], marker="*", s=80, c="tab:red")
        spatial.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        print(f"{shape}: {profile.duration:g}s; max |qd|={np.abs(qd).max(axis=0)}; "
              f"max |qdd|={np.abs(qdd).max(axis=0)}")
    figure.savefig(output / "reference_paths.png", dpi=180)
    figure.savefig(output / "reference_paths.svg")
    plt.close(figure)
    print(output / "reference_paths.png")


if __name__ == "__main__":
    main()
