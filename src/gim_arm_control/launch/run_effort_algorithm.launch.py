"""
Start exactly one torque algorithm node.

The controller manager and MuJoCo/CAN simulator are intentionally launched
separately so switching the algorithm never changes the plant under test.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ALGORITHMS = {
    "pid": ("gim_arm_controller_pid", "cascade_pid_node", "pid.yaml"),
    "lqr": ("gim_arm_controller_lqr", "lqr_node", "lqr.yaml"),
    "mpc": ("gim_arm_controller_mpc", "mpc_node", "mpc.yaml"),
    "smc": ("gim_arm_controller_smc", "smc_node", "smc.yaml"),
}


def generate_launch_description():
    algorithm = LaunchConfiguration("algorithm")
    pid_profile = LaunchConfiguration("pid_profile")
    lqr_profile = LaunchConfiguration("lqr_profile")
    mpc_profile = LaunchConfiguration("mpc_profile")
    lqr_recompute_every = LaunchConfiguration("lqr_recompute_every")
    lqr_tau_penalty_scale = LaunchConfiguration("lqr_tau_penalty_scale")
    lqr_gravity_at_measured = LaunchConfiguration(
        "lqr_gravity_at_measured")
    common_overrides = {
        "autostart": ParameterValue(
            LaunchConfiguration("autostart"), value_type=bool),
        "tau_scale": ParameterValue(
            LaunchConfiguration("tau_scale"), value_type=float),
        "control_hz": ParameterValue(
            LaunchConfiguration("control_hz"), value_type=float),
        "max_track_error_rad": ParameterValue(
            LaunchConfiguration("max_track_error_rad"), value_type=float),
        "max_transition_error_rad": ParameterValue(
            LaunchConfiguration("max_transition_error_rad"), value_type=float),
        "approach_time": ParameterValue(
            LaunchConfiguration("approach_time"), value_type=float),
        "return_time": ParameterValue(
            LaunchConfiguration("return_time"), value_type=float),
        "loops": ParameterValue(
            LaunchConfiguration("loops"), value_type=float),
        "diagnostic_hold": ParameterValue(
            LaunchConfiguration("diagnostic_hold"), value_type=bool),
        "diagnostic_segment": ParameterValue(
            LaunchConfiguration("diagnostic_segment"), value_type=bool),
        "diagnostic_q1_deg": ParameterValue(
            LaunchConfiguration("diagnostic_q1_deg"), value_type=float),
        "diagnostic_q2_deg": ParameterValue(
            LaunchConfiguration("diagnostic_q2_deg"), value_type=float),
        "diagnostic_q3_deg": ParameterValue(
            LaunchConfiguration("diagnostic_q3_deg"), value_type=float),
        "diagnostic_hold_time": ParameterValue(
            LaunchConfiguration("diagnostic_hold_time"), value_type=float),
        "diagnostic_qd1_rad_s": ParameterValue(
            LaunchConfiguration("diagnostic_qd1_rad_s"), value_type=float),
        "diagnostic_qd2_rad_s": ParameterValue(
            LaunchConfiguration("diagnostic_qd2_rad_s"), value_type=float),
        "diagnostic_qd3_rad_s": ParameterValue(
            LaunchConfiguration("diagnostic_qd3_rad_s"), value_type=float),
        "diagnostic_segment_time": ParameterValue(
            LaunchConfiguration("diagnostic_segment_time"), value_type=float),
        "fixed_track_gain_index": ParameterValue(
            LaunchConfiguration("lqr_fixed_track_gain_index"),
            value_type=int),
        "command_heartbeat_nm": ParameterValue(
            LaunchConfiguration("command_heartbeat_nm"), value_type=float),
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool),
        "log_file": LaunchConfiguration("log_file"),
    }

    actions = [
        DeclareLaunchArgument(
            "algorithm", default_value="lqr", choices=list(ALGORITHMS)),
        DeclareLaunchArgument(
            "pid_profile", default_value="matlab_reference",
            choices=["matlab_reference", "gazebo_smooth"],
            description=(
                "PID only: original MATLAB gains or a separately tuned "
                "Gazebo profile")),
        DeclareLaunchArgument(
            "lqr_profile", default_value="matlab_reference",
            choices=["matlab_reference", "safe_100hz"],
            description=(
                "LQR only: exact MATLAB weights or the discretely-stable "
                "100 Hz Gazebo/hardware bring-up profile")),
        DeclareLaunchArgument(
            "mpc_profile", default_value="gazebo_safe",
            choices=["matlab_reference", "gazebo_safe"],
            description=(
                "MPC only: validated MATLAB slew limits or a delay-robust "
                "100 Hz Gazebo bring-up profile")),
        DeclareLaunchArgument(
            "lqr_recompute_every",
            default_value=PythonExpression([
                "'2' if '", lqr_profile,
                "' == 'safe_100hz' else '1'",
            ]),
            description=(
                "LQR only: recompute gain every N torque cycles; use 2 for "
                "a 200 Hz torque loop with 100 Hz gain updates")),
        DeclareLaunchArgument(
            "lqr_tau_penalty_scale",
            default_value=PythonExpression([
                "'512.0' if '", lqr_profile,
                "' == 'safe_100hz' else '1.0'",
            ]),
            description=(
                "LQR only: multiplier on Bryson R; larger values reduce "
                "feedback bandwidth and improve delay robustness")),
        DeclareLaunchArgument(
            "lqr_gravity_at_measured",
            default_value=PythonExpression([
                "'true' if '", lqr_profile,
                "' == 'safe_100hz' else 'false'",
            ]),
            description=(
                "LQR only: true uses G(q measured), false uses MATLAB-style "
                "inverse dynamics entirely at the reference state")),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("tau_scale", default_value=PythonExpression([
            "'1.0' if '", algorithm,
            "' == 'mpc' else ('0.50' if '", algorithm,
            "' == 'pid' and '", pid_profile,
            "' == 'gazebo_smooth' else '0.35')",
        ])),
        DeclareLaunchArgument("control_hz", default_value=PythonExpression([
            "'2000.0' if '", algorithm, "' == 'pid' and '", pid_profile,
            "' == 'gazebo_smooth' else ('200.0' if '", algorithm,
            "' == 'lqr' and '", lqr_profile,
            "' == 'safe_100hz' else '100.0')",
        ])),
        DeclareLaunchArgument("max_track_error_rad", default_value="0.05"),
        DeclareLaunchArgument(
            "max_transition_error_rad", default_value="0.10"),
        DeclareLaunchArgument("approach_time", default_value="5.0"),
        DeclareLaunchArgument("return_time", default_value="5.0"),
        DeclareLaunchArgument("loops", default_value="1.0"),
        DeclareLaunchArgument("diagnostic_hold", default_value="false"),
        DeclareLaunchArgument("diagnostic_segment", default_value="false"),
        DeclareLaunchArgument("diagnostic_q1_deg", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_q2_deg", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_q3_deg", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_hold_time", default_value="15.0"),
        DeclareLaunchArgument("diagnostic_qd1_rad_s", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_qd2_rad_s", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_qd3_rad_s", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_segment_time", default_value="2.0"),
        DeclareLaunchArgument(
            "lqr_fixed_track_gain_index",
            default_value="-1",
            description=(
                "LQR diagnostic segment only: -1=schedule, "
                ">=0=hold this K during TRACK")),
        DeclareLaunchArgument("command_heartbeat_nm", default_value="1.0e-6"),
        DeclareLaunchArgument(
            "lqr_use_armature",
            default_value="true",
            description=(
                "LQR only: include reflected motor inertia in its model")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_file", default_value=""),
    ]
    for name, (package, executable, config) in ALGORITHMS.items():
        config_file = config
        if name == "pid":
            config_file = PythonExpression([
                "'pid_gazebo_smooth.yaml' if '", pid_profile,
                "' == 'gazebo_smooth' else 'pid.yaml'",
            ])
        if name == "lqr":
            config_file = PythonExpression([
                "'lqr_safe_100hz.yaml' if '", lqr_profile,
                "' == 'safe_100hz' else 'lqr.yaml'",
            ])
        if name == "mpc":
            config_file = PythonExpression([
                "'mpc_gazebo_safe.yaml' if '", mpc_profile,
                "' == 'gazebo_safe' else 'mpc.yaml'",
            ])
        parameter_overrides = [common_overrides]
        if name == "lqr":
            parameter_overrides.append({
                "recompute_every": ParameterValue(
                    lqr_recompute_every, value_type=int),
                "tau_penalty_scale": ParameterValue(
                    lqr_tau_penalty_scale, value_type=float),
                "gravity_at_measured": ParameterValue(
                    lqr_gravity_at_measured, value_type=bool),
                "use_armature": ParameterValue(
                    LaunchConfiguration("lqr_use_armature"), value_type=bool),
            })
        actions.append(Node(
            package=package,
            executable=executable,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare(package), "config", config_file]),
                *parameter_overrides,
            ],
            condition=IfCondition(PythonExpression([
                "'", algorithm, "' == '", name, "'",
            ])),
        ))
    return LaunchDescription(actions)
