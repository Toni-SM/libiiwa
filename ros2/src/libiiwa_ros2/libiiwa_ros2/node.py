#!/usr/bin/env python3
from typing import Optional, Mapping

import math
import rclpy
import threading
import sensor_msgs.msg
import geometry_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles

try:
    import libiiwa
except ModuleNotFoundError:
    import os
    import sys
    # get path relative to repository root
    libiiwa_path = os.path.abspath(os.path.join(os.path.dirname(__file__), *(['..'] * 7), 'libiiwa'))
    if os.path.exists(libiiwa_path):
        print('libiiwa path: {}'.format(libiiwa_path))
        sys.path.append(libiiwa_path)
    # get path form env variable LIBIIWA_PATH
    else:
        libiiwa_path = os.environ.get('LIBIIWA_PATH')
        if libiiwa_path is not None:
            print('libiiwa path: {}'.format(libiiwa_path))
            sys.path.append(libiiwa_path)
    try:
        import libiiwa
    except ModuleNotFoundError:
        print('\nlibiiwa not found')
        print('Set environment variable LIBIIWA_PATH to libiiwa.py folder (e.g. export LIBIIWA_PATH=/path/to/libiiwa)')
        print(' or copy libiiwa.py to the same folder as this node\n')
        sys.exit(1)


JOINTS = {"iiwa_joint_1": {"index": 0, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_2": {"index": 1, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_3": {"index": 2, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_4": {"index": 3, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_5": {"index": 4, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_6": {"index": 5, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0},
          "iiwa_joint_7": {"index": 6, "type": "revolute", "has_limits": False, "lower": 0, "upper": 0}}


class Iiwa:
    def __init__(self, 
                 node: Node,
                 interface: libiiwa.LibIiwa, 
                 joints: Mapping[str, dict], 
                 names: Optional[Mapping[str, str]] = {}, 
                 qos_profile: QoSProfile = QoSPresetProfiles.SENSOR_DATA.value,
                 verbose: bool = False) -> None:
        """ROS control interface for the KUKA LBR iiwa robot

        :param node: ROS2 node
        :type node: rclpy.node.Node
        :param interface: libiiwa.LibIiwa object
        :type interface: libiiwa.LibIiwa
        :param joints: Joint names and parameters
        :type joints: Mapping[str, dict]
        :param names: Topics and services mapping to the corresponding ROS names (default: {})
        :type names: Mapping[str, str], optional
        :param qos_profile: The quality of service profile to apply (default: QoSPresetProfiles.SENSOR_DATA)
        :type qos_profile: QoSProfile, optional
        :param verbose: Print additional information (default: False)
        :type verbose: bool, optional
        """
        self._node = node
        self._names = names
        self._joints = joints
        self._interface = interface
        self._qos_profile = qos_profile
        self._num_joints = len(joints)
        self._verbose = verbose

        # publishers
        self._publishers = []
        self._pub_joint_states = None
        self._pub_end_effector_pose = None
        self._pub_end_effector_wrench = None

        # subscribers
        self._subscribers = []
        self._sub_joint_command = None
        self._sub_cartesian_command = None

        # services
        self._services = []

        # messages
        self._msg_joint_states = sensor_msgs.msg.JointState()
        self._msg_end_effector_pose = geometry_msgs.msg.Pose()
        self._msg_end_effector_wrench = geometry_msgs.msg.Wrench()

        # initialize messages
        self._msg_joint_states.name = sorted(list(self._joints.keys()))

    # motion command

    def _callback_joint_command(self, msg: sensor_msgs.msg.JointState) -> None:  # DONE
        names = msg.name
        positions = msg.position

        # validate message
        if not len(names):
            names = self._msg_joint_states.name.copy()
            if len(positions) != self._num_joints:
                self._node.get_logger().error('Number of positions ({}) does not match number of joints ({})' \
                    .format(len(positions), self._num_joints))
                return
        elif len(names) != len(positions):
            self._node.get_logger().error('The message has different number of names ({}) and positions ({})' \
                .format(len(names), len(positions)))
            return

        # parse message
        target_positions = [math.nan] * self._num_joints
        for name, position in zip(names, positions):
            if not name in self._joints:
                self._node.get_logger().error('Invalid joint name: {}'.format(name))
                return
            index = self._joints[name]['index']
            target_positions[index] = position

        # move the robot
        try:
            status = self._interface.command_joint_position(target_positions)
        except Exception as e:
            self._node.get_logger().error('Failed to command joint position to {}'.format(target_positions))
            self._node.get_logger().error(e)
            return

        if not status:
            self._node.get_logger().error('Failed to command joint position to {}'.format(target_positions))
            self._node.get_logger().error(self._interface.get_last_error())
        if self._verbose:
            self._node.get_logger().info('Commanded joint position to {} ({})'.format(target_positions, status))

    def _callback_cartesian_command(self, msg: geometry_msgs.msg.Pose) -> None:  # PARTIAL DONE
        position = msg.position
        quaternion = msg.orientation

        # validate message
        parse_quaternion = True
        if math.isnan(quaternion.x) or math.isnan(quaternion.y) or math.isnan(quaternion.z) or math.isnan(quaternion.w):
            if not (math.isnan(quaternion.x) and math.isnan(quaternion.y) and math.isnan(quaternion.z) and math.isnan(quaternion.w)):
                self._node.get_logger().error('Invalid orientation: {}'.format([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
                return
            parse_quaternion = False

        # convert message to expected format
        position = [position.x, position.y, position.z]
        if parse_quaternion:
            quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
            orientation = [1,2,3]  # TODO: fix convertion
            # orientation = euler_from_quaternion(quaternion, 'sxyz')  # TODO: fix convertion
        else:
            orientation = [math.nan] * 3

        # move the robot
        try:
            status = self._interface.command_cartesian_pose(position, orientation)
        except Exception as e:
            self._node.get_logger().error('Failed to command cartesian pose to {}, {}'.format(position, orientation))
            self._node.get_logger().error(e)
            return

        if not status:
            self._node.get_logger().error('Failed to command cartesian pose to {}, {}'.format(position, orientation))
            self._node.get_logger().error(self._interface.get_last_error())
        if self._verbose:
            self._node.get_logger().info('Commanded cartesian pose to {}, {} ({})'.format(position, orientation, status))

    def start(self) -> None:
        """Start the publisher
        """
        self.stop()

        # create publishers
        self._pub_joint_states = self._node.create_publisher(msg_type=sensor_msgs.msg.JointState,
                                                             topic=self._names.get("joint_states", "/iiwa/joint_states"),
                                                             qos_profile=self._qos_profile)
        self._pub_end_effector_pose = self._node.create_publisher(msg_type=geometry_msgs.msg.Pose,
                                                                  topic=self._names.get("end_effector_pose", "/iiwa/end_effector_pose"),
                                                                  qos_profile=self._qos_profile)
        self._pub_end_effector_wrench = self._node.create_publisher(msg_type=geometry_msgs.msg.Wrench,
                                                                    topic=self._names.get("end_effector_wrench", "/iiwa/end_effector_wrench"),
                                                                    qos_profile=self._qos_profile)

        self._publishers = [self._pub_joint_states,
                            self._pub_end_effector_pose,
                            self._pub_end_effector_wrench]

        # create subscribers
        self._sub_joint_command = self._node.create_subscription(msg_type=sensor_msgs.msg.JointState,
                                                                 topic=self._names.get("joint_command", "/iiwa/command/joint"),
                                                                 callback=self._callback_joint_command,
                                                                 qos_profile=self._qos_profile)
        self._sub_cartesian_command = self._node.create_subscription(msg_type=geometry_msgs.msg.Pose,
                                                                     topic=self._names.get("cartesian_command", "/iiwa/command/cartesian"),
                                                                     callback=self._callback_cartesian_command,
                                                                     qos_profile=self._qos_profile)

        self._subscribers = [self._sub_joint_command,
                             self._sub_cartesian_command]


        # # create services
        # name = self._names.get("set_desired_joint_velocity_rel", "/iiwa/set_desired_joint_velocity_rel")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_joint_velocity_rel))

        # name = self._names.get("set_desired_joint_acceleration_rel", "/iiwa/set_desired_joint_acceleration_rel")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_joint_acceleration_rel))

        # name = self._names.get("set_desired_joint_jerk_rel", "/iiwa/set_desired_joint_jerk_rel")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_joint_jerk_rel))

        # name = self._names.get("set_desired_cartesian_velocity", "/iiwa/set_desired_cartesian_velocity")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_cartesian_velocity))

        # name = self._names.get("set_desired_cartesian_acceleration", "/iiwa/set_desired_cartesian_acceleration")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_cartesian_acceleration))

        # name = self._names.get("set_desired_cartesian_jerk", "/iiwa/set_desired_cartesian_jerk")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetDouble,
        #                                     handler=self._handler_set_desired_cartesian_jerk))

        # name = self._names.get("set_control_interface", "/iiwa/set_control_interface")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetString,
        #                                     handler=self._handler_set_control_interface))

        # name = self._names.get("set_motion_type", "/iiwa/set_motion_type")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetString,
        #                                     handler=self._handler_set_motion_type))

        # name = self._names.get("set_control_mode", "/iiwa/set_control_mode")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetString,
        #                                     handler=self._handler_set_control_mode))

        # name = self._names.get("set_execution_type", "/iiwa/set_execution_type")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetString,
        #                                     handler=self._handler_set_execution_type))

        # name = self._names.get("set_communication_mode", "/iiwa/set_communication_mode")
        # self._services.append(rospy.Service(name=name,
        #                                     service_class=SetString,
        #                                     handler=self._handler_set_communication_mode))

    def stop(self) -> None:
        """Stop the publisher
        """
        for publisher in self._publishers:
            self._node.destroy_publisher(publisher)
        for subscriber in self._subscribers:
            self._node.destroy_subscription(subscriber)

        self._pub_joint_states = None
        self._pub_end_effector_pose = None
        self._pub_end_effector_wrench = None

        self._publishers = []
        self._subscribers = []
        self._services = []

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        state = self._interface.get_state()

        # TODO: set header

        # joint states
        self._msg_joint_states.position = state["joint_position"].tolist()
        self._msg_joint_states.velocity = state["joint_velocity"].tolist()
        self._msg_joint_states.effort = state["joint_torque"].tolist()

        # end-effector pose
        position = state["cartesian_position"]
        self._msg_end_effector_pose.position.x = position[0].item()
        self._msg_end_effector_pose.position.y = position[1].item()
        self._msg_end_effector_pose.position.z = position[2].item()

        # TODO: convert to quaternion
        orientation = state["cartesian_orientation"]
        self._msg_end_effector_pose.orientation.x = orientation[0].item()
        self._msg_end_effector_pose.orientation.y = orientation[1].item()
        self._msg_end_effector_pose.orientation.z = orientation[2].item()
        self._msg_end_effector_pose.orientation.w = orientation[0].item()

        # end-effector wrench
        force = state["cartesian_force"]
        self._msg_end_effector_wrench.force.x = force[0].item()
        self._msg_end_effector_wrench.force.y = force[1].item()
        self._msg_end_effector_wrench.force.z = force[2].item()

        torque = state["cartesian_torque"]
        self._msg_end_effector_wrench.torque.x = torque[0].item()
        self._msg_end_effector_wrench.torque.y = torque[1].item()
        self._msg_end_effector_wrench.torque.z = torque[2].item()

        # publish
        self._pub_joint_states.publish(self._msg_joint_states)
        self._pub_end_effector_pose.publish(self._msg_end_effector_pose)
        self._pub_end_effector_wrench.publish(self._msg_end_effector_wrench)




def main():
    
    from libiiwa import LibIiwa

    # init node
    rclpy.init()
    node = Node(node_name="iiwa", 
                allow_undeclared_parameters=True, 
                automatically_declare_parameters_from_overrides=True)
    rate = node.create_rate(50)  # Hz

    # get launch parameters
    robot_name = node.get_parameter('robot_name').get_parameter_value().string_value
    controller_name = node.get_parameter('controller_name').get_parameter_value().string_value
    action_namespace = node.get_parameter('action_namespace').get_parameter_value().string_value
    libiiwa_ip = node.get_parameter('libiiwa_ip').get_parameter_value().string_value
    libiiwa_port = node.get_parameter('libiiwa_port').get_parameter_value().integer_value
    run_without_communication = node.get_parameter('run_without_communication').get_parameter_value().bool_value
    verbose = node.get_parameter('verbose').get_parameter_value().bool_value
    

    # init robot interface
    robot = LibIiwa(ip=libiiwa_ip, port=libiiwa_port, run_without_communication=run_without_communication)
    robot.start()

    robot.set_control_interface(libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO)
    robot.set_desired_joint_velocity_rel(0.5)

    # load controllers
    controllers = [Iiwa(node, robot, JOINTS, verbose=verbose),
                #    FollowJointTrajectory(node, robot, f"/{controller_name}/{action_namespace}", JOINTS)]
    ]

    # control loop
    def control_loop(node):
        rate = node.create_rate(50)  # Hz

        for controller in controllers:
            controller.start()

        while rclpy.ok():
            robot.get_state(refresh=True)
            for controller in controllers:
                controller.step(0)
            rate.sleep()

        for controller in controllers:
            controller.stop()

    # start control loop
    threading.Thread(target=control_loop, args=(node,)).start()
    
    rclpy.spin(node)