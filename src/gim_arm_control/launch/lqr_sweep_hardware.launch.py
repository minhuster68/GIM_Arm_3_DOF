"""Run LQR on the real arm with circle, R or A references."""

from gim_control.hardware_trajectory_launch import generate_hardware_launch


def generate_launch_description():
    return generate_hardware_launch('lqr')
