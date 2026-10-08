from gim_control.effort_controller_node import run_controller

from .factory import PositionPidFactory


def main():
    run_controller(PositionPidFactory())
