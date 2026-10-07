#!/usr/bin/env python3
"""Run the LQR Gazebo profile on an isolated ROS domain and save results."""

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

from gim_arm_controller_lqr.controller import LqrController
from gim_arm_controller_lqr.factory import LqrFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, plot, print_metrics


ROOT = Path(__file__).resolve().parents[1]


def run_case(mass, output, tau_penalty_scale=None):
    name = "noload" if mass == 0.0 else "payload_" + str(mass).replace(".", "p")
    scale = 0.50 if mass == 0.0 else 0.75
    csv_path = output / f"{name}.csv"
    gazebo_log = (output / f"{name}_gazebo.log").open("w")
    gazebo = subprocess.Popen([
        "ros2", "launch", "gim_control", "gazebo_effort_control.launch.py",
        "gui:=false", "verbose:=false", f"payload_mass_kg:={mass}"],
        stdout=gazebo_log, stderr=subprocess.STDOUT, start_new_session=True)
    executor = probe = lqr = None
    active_effort = False
    try:
        ros_args = [
            "--ros-args", "--params-file", str(ROOT / "src" /
                "gim_arm_controller_lqr/config/lqr_gazebo_matlab.yaml"),
            "-p", "use_sim_time:=false", "-p", f"log_file:={csv_path}",
            "-p", "max_track_error_rad:=0.05",
            "-p", "max_transition_error_rad:=0.10",
            "-p", f"tau_scale:={scale}"]
        if tau_penalty_scale is not None:
            ros_args += ["-p", f"tau_penalty_scale:={tau_penalty_scale}"]
        rclpy.init(args=ros_args)
        executor = SingleThreadedExecutor()
        probe = Node("lqr_gazebo_validation")
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
        lqr = EffortControllerNode(LqrFactory())
        assert isinstance(lqr.controller, LqrController)
        executor.add_node(lqr)
        deadline = time.monotonic() + 120.0
        while lqr.phase == "WAIT":
            if time.monotonic() > deadline:
                raise TimeoutError("No joint states")
            executor.spin_once(timeout_sec=0.01)
        change_effort(True)
        active_effort = True
        spin_for(5.0)
        lqr.set_parameters([Parameter("autostart", value=True)])
        phases = set()
        deadline = time.monotonic() + 180.0
        while lqr.phase not in ("HOLD", "ABORT"):
            if time.monotonic() > deadline:
                raise TimeoutError("Trajectory did not finish")
            executor.spin_once(timeout_sec=0.01)
            phases.add(lqr.phase)
        passed = lqr.phase == "HOLD" and {"APPROACH", "TRACK", "RETURN"} <= phases
        result = {
            "case": name, "payload_mass_kg": mass, "tau_scale": scale,
            "final_phase": lqr.phase, "phases": sorted(phases),
            "samples": len(lqr.rows), "passed": passed,
            "control_hz": 1.0 / lqr.dt_nom,
            "profile": "gazebo_matlab",
            "gain_schedule_mode": lqr.controller.gain_schedule_mode,
            "gain_samples": len(lqr.controller._gain_schedule_gains),
            "max_int_e": list(lqr.controller.weights.max_int_e),
            "max_e": list(lqr.controller.weights.max_e),
            "matlab_branch": "position-tracking",
            "matlab_commit": "addec462971d946519cafa6cda5ebbcd18539da2",
            "position_tracking_only": lqr.controller.weights.position_tracking_only,
            "q_velocity": np.diag(lqr.controller.Q)[-lqr.n:].tolist(),
            "integral_limit": lqr.controller.i_limit,
            "use_discrete_lqr": lqr.controller.use_discrete_lqr,
            "tau_penalty_scale": lqr.controller.weights.tau_penalty_scale,
            "tau_ff": "inverse_dynamics(q_ref, qd_ref, qdd_ref)",
        }
        print(json.dumps(result), flush=True)
        change_effort(False)
        active_effort = False
        lqr.dump()
        (output / f"{name}_validation.json").write_text(
            json.dumps(result, indent=2) + "\n")
        if csv_path.exists():
            log = load_log(csv_path)
            error = log["q"] - log["q_ref"]
            track = log["phase"] == "TRACK"
            result["rms_error_deg"] = np.degrees(np.sqrt(np.mean(error**2, axis=0))).tolist()
            result["max_error_deg"] = np.degrees(np.max(np.abs(error), axis=0)).tolist()
            if np.any(track):
                result["track_rms_error_deg"] = np.degrees(np.sqrt(np.mean(error[track]**2, axis=0))).tolist()
            result["max_torque_nm"] = np.max(np.abs(log["tau"]), axis=0).tolist()
            print_metrics(log)
            plot(log, str(output / f"{name}_joints.png"), False, 30000)
        (output / f"{name}_validation.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        if lqr is not None:
            if active_effort:
                try:
                    change_effort(False)
                except Exception as error:
                    print(f"Cleanup switch: {error}", flush=True)
            lqr.dump()
            executor.remove_node(lqr)
            lqr.destroy_node()
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
                        default=ROOT / "results/lqr_matlab_gazebo")
    parser.add_argument("--ros-domain-id", type=int, default=74)
    parser.add_argument("--gazebo-port", type=int, default=11374)
    parser.add_argument("--payload-mass-kg", type=float, nargs="+", default=[0.0, 0.5])
    parser.add_argument("--tau-penalty-scale", type=float)
    args = parser.parse_args()
    if any(not np.isfinite(mass) or mass < 0.0 for mass in args.payload_mass_kg):
        parser.error("Payload masses must be finite and nonnegative")
    if args.tau_penalty_scale is not None and (
            not np.isfinite(args.tau_penalty_scale) or args.tau_penalty_scale <= 0.0):
        parser.error("tau-penalty-scale must be finite and positive")
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
    results = [run_case(mass, output, args.tau_penalty_scale)
               for mass in args.payload_mass_kg]
    (output / "validation.json").write_text(json.dumps(results, indent=2) + "\n")
    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
