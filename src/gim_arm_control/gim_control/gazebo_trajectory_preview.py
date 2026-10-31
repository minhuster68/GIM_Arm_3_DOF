"""Show a reference in Gazebo and follow q/qd/qdd with its position controller.

This is a geometric preview, not a PID/LQR/MPC/SMC performance experiment.
Motion is gated on a GazeboSystem robot_description, never CAN hardware.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, SwitchController
from gazebo_msgs.srv import SpawnEntity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from gim_control import sweep_trajectory as sweep
from gim_control.gazebo_description import GAZEBO_BASE_HEIGHT_M
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.reference_trajectory import Quintic, load_trajectory


def sample_motion(profile, home, approach_time=5., return_time=5., sample_dt=.02):
    """Compose approach/path/return, retaining analytic derivatives at joins."""
    if not np.isfinite([approach_time, return_time, sample_dt]).all() or min(
            approach_time, return_time, sample_dt) <= 0:
        raise ValueError('Durations and sample_dt must be finite and positive')
    segments = [
        (Quintic(home, profile.at(0.)[0], approach_time), approach_time, 'APPROACH'),
        (profile, profile.duration, 'TRACK'),
        (Quintic(profile.at(profile.duration)[0], home, return_time), return_time, 'RETURN'),
    ]
    times, states, phases = [], [], []
    elapsed = 0.
    for index, (segment, duration, phase) in enumerate(segments):
        local = np.linspace(0., duration, int(np.ceil(duration/sample_dt))+1)
        if index:
            local = local[1:]
        for t in local:
            times.append(elapsed+t)
            states.append(segment.at(float(t)))
            phases.append(phase)
        elapsed += duration
    q, qd, qdd = (np.asarray([state[i] for state in states]) for i in range(3))
    return np.asarray(times), q, qd, qdd, np.asarray(phases)


def reference_sdf(positions, name='square_reference_path'):
    """Collision-free static green line, in Gazebo's translated world frame."""
    points = np.asarray(positions, dtype=float).copy()
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError('positions must be N x 3 with N >= 2')
    if not np.isfinite(points).all():
        raise ValueError('positions must be finite')
    points[:, 2] += GAZEBO_BASE_HEIGHT_M
    root = ET.Element('sdf', {'version': '1.6'})
    model = ET.SubElement(root, 'model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', {'name': 'reference'})
    for i, (start, end) in enumerate(zip(points[:-1], points[1:])):
        delta = end-start
        length = float(np.linalg.norm(delta))
        if length < 1e-8:
            continue
        center = (start+end)/2
        pitch = math.atan2(float(np.linalg.norm(delta[:2])), float(delta[2]))
        yaw = math.atan2(float(delta[1]), float(delta[0]))
        visual = ET.SubElement(link, 'visual', {'name': f'path_{i}'})
        ET.SubElement(visual, 'pose').text = ' '.join(map(str, [*center, 0., pitch, yaw]))
        cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
        ET.SubElement(cylinder, 'radius').text = '.0025'
        ET.SubElement(cylinder, 'length').text = str(length)
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.05 0.9 0.2 1'
        ET.SubElement(material, 'diffuse').text = '0.05 0.9 0.2 1'
        ET.SubElement(material, 'emissive').text = '0.02 0.25 0.05 1'
    return ET.tostring(root, encoding='unicode')


class GazeboTrajectoryPreview(Node):
    def __init__(self):
        super().__init__('gazebo_trajectory_preview')
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('trajectory_shape', 'square')
        self.declare_parameter('approach_time', 5.)
        self.declare_parameter('return_time', 5.)
        self.declare_parameter('repeat', True)
        urdf = self.get_parameter('urdf_file').value
        shape = self.get_parameter('trajectory_shape').value
        self.profile = load_trajectory(urdf, '', self.get_logger(), shape)
        self.kin = GimArmKinematics(urdf, tool_offset_xyz=sweep.TOOL_OFFSET)
        self.names = self.kin.joint_names
        q, _, _ = self.profile.at(np.linspace(0., self.profile.duration, 241))
        self.sdf = reference_sdf([self.kin.fk_position(value) for value in q])
        self.q = None
        self.gazebo_description = False
        self.state = 'WAIT_GAZEBO'
        self.future = None
        self.cycles = 0
        self.create_subscription(JointState, '/joint_states', self.on_state, 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, qos)
        self.listing = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.action = ActionClient(self, FollowJointTrajectory,
                                   '/gim_arm_group_controller/follow_joint_trajectory')
        self.create_timer(.5, self.step)
        self.get_logger().info('Waiting for GazeboSystem, joint states and controllers; no CAN commands.')

    def on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
            self.gazebo_description = any(
                plugin.text == 'gazebo_ros2_control/GazeboSystem'
                for plugin in root.findall('.//ros2_control/hardware/plugin'))
        except ET.ParseError:
            self.gazebo_description = False

    def on_state(self, msg):
        if all(name in msg.name for name in self.names):
            q = np.asarray([msg.position[msg.name.index(name)] for name in self.names])
            if np.isfinite(q).all():
                self.q = q

    def fail(self, message):
        self.state = 'ERROR'
        self.get_logger().error(message)

    def step(self):
        if self.state in ('ERROR', 'DONE', 'RUNNING', 'GOAL_PENDING'):
            return
        # A process on a different ROS graph can never pass this description gate.
        if not self.gazebo_description or self.q is None:
            return
        if self.state == 'WAIT_GAZEBO':
            if not (self.listing.service_is_ready() and self.switch.service_is_ready()
                    and self.spawn.service_is_ready() and self.action.server_is_ready()):
                return
            self.future = self.listing.call_async(ListControllers.Request())
            self.state = 'LISTING'
        elif self.state == 'LISTING' and self.future.done():
            result = self.future.result()
            active = [c.name for c in result.controller if c.state == 'active']
            if 'joint_state_broadcaster' not in active:
                self.state = 'WAIT_GAZEBO'
                return
            loaded = {c.name for c in result.controller}
            if 'gim_arm_group_controller' not in loaded:
                self.state = 'WAIT_GAZEBO'
                return
            self.to_deactivate = [name for name in (
                'forward_position_controller', 'gim_arm_effort_controller') if name in active]
            self.already_active = 'gim_arm_group_controller' in active
            request = SpawnEntity.Request()
            request.name, request.xml, request.reference_frame = 'square_reference_path', self.sdf, 'world'
            self.future = self.spawn.call_async(request)
            self.state = 'SPAWNING'
        elif self.state == 'SPAWNING' and self.future.done():
            result = self.future.result()
            if not result.success:
                self.fail(f'Cannot spawn path: {result.status_message}')
                return
            if self.already_active:
                self.send_motion()
                return
            request = SwitchController.Request()
            request.activate_controllers = ['gim_arm_group_controller']
            request.deactivate_controllers = self.to_deactivate
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 10
            self.future = self.switch.call_async(request)
            self.state = 'SWITCHING'
        elif self.state == 'SWITCHING' and self.future.done():
            if not self.future.result().ok:
                self.fail('Gazebo position trajectory controller switch failed')
                return
            self.send_motion()
        elif self.state == 'NEXT_CYCLE':
            self.send_motion()

    def send_motion(self):
        if not self.gazebo_description:
            self.fail('Gazebo description gate lost')
            return
        times, q, qd, qdd, _ = sample_motion(
            self.profile, self.q.copy(), self.get_parameter('approach_time').value,
            self.get_parameter('return_time').value)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.names
        # Gazebo's constrained-joint solver leaves small velocity jitter at rest.
        # Keep a tight position target, but allow this numerical residual when
        # completing the visualization goal. These tolerances are preview-only.
        goal.goal_tolerance = [
            JointTolerance(name=name, position=.001, velocity=.05)
            for name in self.names
        ]
        goal.goal_time_tolerance.sec = 2
        for t, position, speed, accel in zip(times, q, qd, qdd):
            point = JointTrajectoryPoint()
            point.positions = position.tolist()
            point.velocities = speed.tolist()
            point.accelerations = accel.tolist()
            ns = int(round((float(t)+.15)*1e9))
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)
            goal.trajectory.points.append(point)
        self.state = 'GOAL_PENDING'
        self.future = self.action.send_goal_async(goal)
        self.future.add_done_callback(self.on_goal)

    def on_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail('Gazebo trajectory goal rejected')
            return
        self.state = 'RUNNING'
        self.cycles += 1
        self.get_logger().info(f'Preview cycle {self.cycles}: APPROACH -> SQUARE (27s) -> RETURN; '
                               'green=reference, red sphere=tool. q/qd/qdd included.')
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result()
        if (result.status != GoalStatus.STATUS_SUCCEEDED
                or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            self.fail(f'Gazebo goal failed: {result.result.error_string}')
            return
        self.get_logger().info(f'Preview cycle {self.cycles} completed and returned HOME.')
        self.state = 'NEXT_CYCLE' if self.get_parameter('repeat').value else 'DONE'


def main():
    rclpy.init()
    node = None
    try:
        node = GazeboTrajectoryPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
