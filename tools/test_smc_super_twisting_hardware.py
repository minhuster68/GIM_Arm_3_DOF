#!/usr/bin/env python3
"""Run one STA joint trial through the existing CAN handoff/restore workflow."""

import argparse
import os
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
sys.path.insert(0, str(ROOT / 'src/gim_arm_controller_smc'))
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'


def main(argv=None):
    from gim_control.smc_joint_trial import JOINTS, RosSmcJointTrial, checked_trial

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', choices=JOINTS, default='elbow')
    parser.add_argument('--degrees', type=float, default=3.)
    parser.add_argument('--move', type=float, default=6.)
    parser.add_argument('--hold', type=float, default=3.)
    parser.add_argument('--params-file', type=Path, default=ROOT /
                        'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml')
    parser.add_argument('--log-dir', type=Path,
                        default=ROOT / 'results_new/smc_super_twisting/hardware_joint_trials')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    params = yaml.safe_load(args.params_file.read_text())['smc_controller']['ros__parameters']
    if (params.get('smc_variant') != 'super_twisting'
            or params.get('control_hz') != 100. or params.get('use_sim_time') is not False):
        parser.error('Cần profile super_twisting, control_hz=100, use_sim_time=false')
    values = checked_trial(args.degrees, args.move, args.hold)
    print(f'STA 100 Hz | {args.joint} | HOME + {values[0]:g} deg | '
          f'move={values[1]:g}s, hold={values[2]:g}s | {args.params_file}')
    if args.dry_run:
        print('DRY RUN: không mở ROS/CAN, không chuyển mode hoặc gửi torque.')
        return
    RosSmcJointTrial(args.log_dir, show_plot=not args.no_show).run(
        JOINTS.index(args.joint), *values, params)


if __name__ == '__main__':
    main()
