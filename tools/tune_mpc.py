#!/usr/bin/env python3
"""Terminal tuning for the MPC hardware branch."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'

from gim_control.algorithm_tuner import main

if __name__ == '__main__':
    main('mpc', ROOT)
