#!/usr/bin/env python3
"""Selected-joint MPC tuning; run through tune_mpc_terminal.sh."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'

from gim_control.mpc_joint_tuner import main

if __name__ == '__main__':
    main(ROOT)
