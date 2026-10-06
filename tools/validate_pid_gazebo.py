#!/usr/bin/env python3
"""Run the PID Gazebo profile on an isolated ROS domain and save results."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from gim_arm_controller_pid.controller import PositionPidController
from gim_arm_controller_pid.factory import CascadePidFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, plot, print_metrics


ROOT = Path(__file__).resolve().parents[1]


def run_case(mass, output):
    name = "noload" if mass == 0.0 else "payload_0p5"
    scale = 0.50 if mass == 0.0 else 0.75
    csv_path = output / f"{name}.csv"
    gazebo_log = (output / f"{name}_gazebo.log").open("w")
    gazebo = subprocess.Popen([
        "ros2", "launch", "gim_control", "gazebo_effort_control.launch.py",
        "gui:=false", "verbose:=false", f"payload_mass_kg:={mass}"],
        stdout=gazebo_log, stderr=subprocess.STDOUT, start_new_session=True)
    executor = probe = pid = None
    active_effort = False
    try:
        rclpy.init(args=[
            "--ros-args", "--params-file", str(ROOT / "src" /
                "gim_arm_controller_pid/config/pid_gazebo_smooth.yaml"),
            "-p", "use_sim_time:=false", "-p", f"log_file:={csv_path}",
            "-p", "max_track_error_rad:=0.05",
            "-p", "max_transition_error_rad:=0.10",
            "-p", f"tau_scale:={scale}"])
        executor = SingleThreadedExecutor()
        probe = Node("pid_gazebo_validation")
        executor.add_node(probe)
        listing = probe.create_client(
            ListControllers, "/controller_manager/list_controllers")
        switch = probe.create_client(
            SwitchController, "/controller_manager/switch_controller")

        def call(client, request, timeout=15.0):
            deadline = time.monotonic() + timeout
            while not client.service_is_ready():
                if time.monotonic() > deadline:
                    raise TimeoutError(client.srv_name)
                executor.spin_once(timeout_sec=0.02)
            future = client.call_async(request)
            while not future.done():
                if time.monotonic() > deadline:
                    raise TimeoutError(client.srv_name)
                executor.spin_once(timeout_sec=0.02)
            return future.result()

        def spin_for(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)

        def change_effort(enable):
            request = SwitchController.Request()
            request.activate_controllers = [
                "gim_arm_effort_controller" if enable
                else "forward_position_controller"]
            request.deactivate_controllers = [
                "forward_position_controller" if enable
                else "gim_arm_effort_controller"]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 10
            if not call(switch, request).ok:
                raise RuntimeError("Controller switch failed")

        deadline = time.monotonic() + 45.0
        while True:
            if gazebo.poll() is not None:
                raise RuntimeError("Gazebo exited during startup")
            controllers = call(listing, ListControllers.Request(), 45.0)
            states = {item.name: item.state for item in controllers.controller}
            if (states.get("gim_arm_effort_controller") == "inactive"
                    and states.get("gim_arm_group_controller") == "inactive"
                    and states.get("forward_position_controller") == "active"):
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Gazebo controller startup")
            executor.spin_once(timeout_sec=0.05)
        spin_for(2.0)  # Let launch command HOME and enable gravity.
        pid = EffortControllerNode(CascadePidFactory())
        assert isinstance(pid.controller, PositionPidController)
        executor.add_node(pid)
        deadline = time.monotonic() + 10.0
        while pid.phase == "WAIT":
            if time.monotonic() > deadline:
                raise TimeoutError("No joint states")
            executor.spin_once(timeout_sec=0.01)
        change_effort(True)
        active_effort = True
        spin_for(5.0)
        pid.set_parameters([Parameter("autostart", value=True)])
        phases = set()
        deadline = time.monotonic() + 180.0
        while pid.phase not in ("HOLD", "ABORT"):
            if time.monotonic() > deadline:
                raise TimeoutError("Trajectory did not finish")
            executor.spin_once(timeout_sec=0.01)
            phases.add(pid.phase)
        passed = pid.phase == "HOLD" and {"APPROACH", "TRACK", "RETURN"} <= phases
        result = {
            "case": name, "payload_mass_kg": mass, "tau_scale": scale,
            "final_phase": pid.phase, "phases": sorted(phases),
            "samples": len(pid.rows), "passed": passed,
            "control_hz": 1.0 / pid.dt_nom,
            "kp": pid.controller.kp.tolist(),
            "ki": pid.controller.ki.tolist(),
            "kd": pid.controller.kd.tolist(),
            "integral_limit": pid.controller.integral_limit.tolist(),
            "tau_ff": "inverse_dynamics(q_ref, qd_ref, qdd_ref)",
        }
        print(json.dumps(result), flush=True)
        change_effort(False)
        active_effort = False
        pid.dump()
        (output / f"{name}_validation.json").write_text(
            json.dumps(result, indent=2) + "\n")
        if csv_path.exists():
            log = load_log(csv_path)
            print_metrics(log)
            plot(log, str(output / f"{name}_joints.png"), False, 30000)
        return result
    finally:
        if pid is not None:
            if active_effort:
                try:
                    change_effort(False)
                except Exception as error:
                    print(f"Cleanup switch: {error}", flush=True)
            pid.dump()
            executor.remove_node(pid)
            pid.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        if gazebo.poll() is None:
            os.killpg(gazebo.pid, signal.SIGINT)
            try:
                gazebo.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(gazebo.pid, signal.SIGTERM)
                try:
                    gazebo.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    os.killpg(gazebo.pid, signal.SIGKILL)
                    gazebo.wait(timeout=5.0)
        gazebo_log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "results/pid_inverse_dynamics_gazebo")
    parser.add_argument("--ros-domain-id", type=int, default=73)
    parser.add_argument("--gazebo-port", type=int, default=11373)
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be between 0 and 232")
    if not 1 <= args.gazebo_port <= 65535:
        parser.error("Gazebo port must be between 1 and 65535")
    with socket.socket() as connection:
        connection.settimeout(1.0)
        if connection.connect_ex(("127.0.0.1", args.gazebo_port)) == 0:
            parser.error("Gazebo port already in use; choose another port/domain")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(
        ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY="1",
        ROS_LOG_DIR=str(output / "ros_logs"),
        GAZEBO_MASTER_URI=f"http://127.0.0.1:{args.gazebo_port}",
        MPLCONFIGDIR=str(output / "matplotlib_cache"),
        RCUTILS_COLORIZED_OUTPUT="0")
    results = [run_case(mass, output) for mass in (0.0, 0.5)]
    (output / "validation.json").write_text(json.dumps(results, indent=2) + "\n")
    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
