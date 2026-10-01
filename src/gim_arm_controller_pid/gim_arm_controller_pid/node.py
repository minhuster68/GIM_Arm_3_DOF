from gim_control.effort_controller_node import run_controller

from ..resource.factory import CascadePidFactory


def main():
    run_controller(CascadePidFactory())
