from gim_control.effort_controller_node import run_controller

from .factory import MpcFactory


def main():
    run_controller(MpcFactory())
