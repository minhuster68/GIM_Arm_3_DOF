#!/usr/bin/env python3
"""Run the MPC Gazebo profile on an isolated ROS domain and save results."""

import argparse
import csv
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
from std_srvs.srv import SetBool

from gim_arm_controller_mpc.controller import MpcController
from gim_arm_controller_mpc.factory import MpcFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, plot, print_metrics


ROOT = Path(__file__).resolve().parents[1]


def run_case(mass, output, config_file):
    name = "noload" if mass == 0.0 else "payload_" + str(mass).replace(".", "p")
    scale = 1.0
    csv_path = output / f"{name}.csv"
    gazebo_log = (output / f"{name}_gazebo.log").open("w")
    gazebo = subprocess.Popen([
        "ros2", "launch", "gim_control", "gazebo_effort_control.launch.py",
        "gui:=false", "verbose:=false", f"payload_mass_kg:={mass}"],
        stdout=gazebo_log, stderr=subprocess.STDOUT, start_new_session=True)
    executor = probe = mpc = None
    active_effort = False
    try:
        ros_args = [
            "--ros-args", "--params-file", str(config_file),
            "-p", "use_sim_time:=false", "-p", f"log_file:={csv_path}",
            "-p", "max_track_error_rad:=0.05",
            "-p", "max_transition_error_rad:=0.10",
            "-p", f"tau_scale:={scale}"]
        rclpy.init(args=ros_args)
        executor = SingleThreadedExecutor()
        probe = Node("mpc_gazebo_validation")
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
        mpc = EffortControllerNode(MpcFactory())
        assert isinstance(mpc.controller, MpcController)
        executor.add_node(mpc)
        deadline = time.monotonic() + 120.0
        while mpc.phase == "WAIT":
            if time.monotonic() > deadline:
                raise TimeoutError("No joint states")
            executor.spin_once(timeout_sec=0.01)
        if mpc.phase != "GRAVITY":
            raise RuntimeError("MPC reference preparation failed")
        clutch = probe.create_client(SetBool, "/gim_arm/set_hand_guiding")
        request = SetBool.Request()
        request.data = False
        if not call(clutch, request).success:
            raise RuntimeError("Could not latch HOME before switching controllers")
        change_effort(True)
        active_effort = True
        spin_for(5.0)
        mpc.set_parameters([Parameter("autostart", value=True)])
        phases = set()
        deadline = time.monotonic() + 180.0
        while mpc.phase not in ("HOLD", "ABORT"):
            if time.monotonic() > deadline:
                raise TimeoutError("Trajectory did not finish")
            executor.spin_once(timeout_sec=0.01)
            phases.add(mpc.phase)
        passed = mpc.phase == "HOLD" and {"APPROACH", "TRACK", "RETURN"} <= phases
        result = {
            "case": name, "payload_mass_kg": mass, "tau_scale": scale,
            "final_phase": mpc.phase, "phases": sorted(phases),
            "samples": len(mpc.rows), "passed": passed,
            "control_hz": 1.0 / mpc.dt_nom,
            "use_armature": bool(mpc.get_parameter("use_armature").value),
            "profile": "gazebo_position",
            "config_file": str(config_file),
            "matlab_branch": "position-tracking",
            "matlab_commit": "addec462971d946519cafa6cda5ebbcd18539da2",
            "prediction_horizon": mpc.controller.np,
            "control_horizon": mpc.controller.nc,
            "q_velocity": np.diag(mpc.controller.Q)[-mpc.n:].tolist(),
            "Q_diagonal": np.diag(mpc.controller.Q).tolist(),
            "R_diagonal": np.diag(mpc.controller.R).tolist(),
            "Rd_diagonal": np.diag(mpc.controller.Rd).tolist(),
            "feedback_bounds_mode": mpc.controller.feedback_bounds_mode,
            "feedback_min": mpc.controller.feedback_min.tolist(),
            "feedback_max": mpc.controller.feedback_max.tolist(),
            "torque_slew_rate": mpc.controller.slew.tolist(),
            "qp_solver": mpc.controller.qp_solver,
            "torque_penalty_scale": float(mpc.get_parameter("torque_penalty_scale").value),
            "precompute_prediction": mpc.controller.precompute_prediction,
            "tau_ff": "inverse_dynamics(q_ref, qd_ref, qdd_ref)",
        }
        print(json.dumps(result), flush=True)
        change_effort(False)
        active_effort = False
        mpc.dump()
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
            with csv_path.open(newline="") as stream:
                diagnostics = list(csv.DictReader(stream))
            controls = [r for r in diagnostics if r["event"] == "CONTROL"]
            if controls:
                compute = np.array([float(r["compute_time"]) for r in controls])
                violations = np.array([float(r["constraint_violation"]) for r in controls])
                result["solver_success_fraction"] = float(np.mean([int(r["solver_success"]) for r in controls]))
                result["solver_accepted_fraction"] = float(np.mean([int(r["solver_accepted"]) for r in controls]))
                result["compute_ms_p50_p95_p99_max"] = (np.quantile(compute, [0.50, 0.95, 0.99, 1.0])*1000).tolist()
                result["max_qp_constraint_violation"] = float(np.max(violations))
            print_metrics(log)
            plot(log, str(output / f"{name}_joints.png"), False, 30000)
        (output / f"{name}_validation.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        if mpc is not None:
            if active_effort:
                try:
                    change_effort(False)
                except Exception as error:
                    print(f"Cleanup switch: {error}", flush=True)
            mpc.dump()
            executor.remove_node(mpc)
            mpc.destroy_node()
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
                        default=ROOT / "results/mpc_position_gazebo")
    parser.add_argument("--ros-domain-id", type=int, default=75)
    parser.add_argument("--gazebo-port", type=int, default=11375)
    parser.add_argument("--payload-mass-kg", type=float, nargs="+", default=[0.0, 0.5])
    parser.add_argument("--config", type=Path, default=ROOT / "src" /
                        "gim_arm_controller_mpc/config/mpc_gazebo_position.yaml")
    args = parser.parse_args()
    if any(not np.isfinite(mass) or mass < 0.0 for mass in args.payload_mass_kg):
        parser.error("Payload masses must be finite and nonnegative")
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
    results = [run_case(mass, output, args.config.resolve())
               for mass in args.payload_mass_kg]
    (output / "validation.json").write_text(json.dumps(results, indent=2) + "\n")
    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
