from gim_control.effort_controller_node import run_controller

from .factory import LqrFactory


def main():
    run_controller(LqrFactory())
