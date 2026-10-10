#!/usr/bin/env python3
"""Selected-joint SMC terminal; importing does not open ROS/CAN."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
sys.path.insert(0, str(ROOT / 'src/gim_arm_controller_smc'))
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'

from gim_control.smc_joint_tuner import main

if __name__ == '__main__':
    main(ROOT)
