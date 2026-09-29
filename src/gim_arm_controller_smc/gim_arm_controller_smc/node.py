from gim_control.effort_controller_node import run_controller

from .factory import SmcFactory


def main():
    run_controller(SmcFactory())
