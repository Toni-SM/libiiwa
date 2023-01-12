#!/usr/bin/env python3
from typing import Optional, Mapping

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles
from rclpy.exceptions import ROSInterruptException

import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import Empty

from libiiwa_msgs.srv import SetArray
from libiiwa_msgs.srv import SetNumber
from libiiwa_msgs.srv import SetString
from libiiwa_msgs.srv import SetXYZABC
from libiiwa_msgs.srv import SetXYZABCParam
from libiiwa_msgs.srv import GetError
from libiiwa_msgs.srv import GetBool
from libiiwa_msgs.srv import GetNumber

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
        self._sub_stop_command = None
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

    def _log_info(self, msg):
        self._node.get_logger().info(msg)
    
    def _log_warn(self, msg):
        self._node.get_logger().warn(msg)
    
    def _log_error(self, msg):
        self._node.get_logger().error(msg)

    # motion command

    def _callback_stop_command(self, msg: std_msgs.msg.Empty) -> None:
        # stop the robot
        try:
            status = self._interface.command_stop()
        except Exception as e:
            self._log_error('Failed to stop the robot')
            self._log_error(str(e))
            return

        if not status:
            self._log_error('Failed to stop the robot')
            self._log_error(str(self._interface.get_last_error()))
        if self._verbose:
            self._log_info('Stop robot')

    def _callback_joint_command(self, msg: sensor_msgs.msg.JointState) -> None:  # DONE
        names = msg.name
        positions = msg.position

        # validate message
        if not len(names):
            names = self._msg_joint_states.name.copy()
            if len(positions) != self._num_joints:
                self._log_error('Number of positions ({}) does not match number of joints ({})' \
                    .format(len(positions), self._num_joints))
                return
        elif len(names) != len(positions):
            self._log_error('The message has different number of names ({}) and positions ({})' \
                .format(len(names), len(positions)))
            return

        # parse message
        target_positions = [math.nan] * self._num_joints
        for name, position in zip(names, positions):
            if not name in self._joints:
                self._log_error('Invalid joint name: {}'.format(name))
                return
            index = self._joints[name]['index']
            target_positions[index] = position

        # move the robot
        try:
            status = self._interface.command_joint_position(target_positions)
        except Exception as e:
            self._log_error('Failed to command joint position to {}'.format(target_positions))
            self._log_error(str(e))
            return

        if not status:
            self._log_error('Failed to command joint position to {}'.format(target_positions))
            self._log_error(str(self._interface.get_last_error()))
        if self._verbose:
            self._log_info('Commanded joint position to {} ({})'.format(target_positions, status))

    def _callback_cartesian_command(self, msg: geometry_msgs.msg.Pose) -> None:  # PARTIAL DONE
        position = msg.position
        quaternion = msg.orientation

        # validate message
        parse_quaternion = True
        if math.isnan(quaternion.x) or math.isnan(quaternion.y) or math.isnan(quaternion.z) or math.isnan(quaternion.w):
            if not (math.isnan(quaternion.x) and math.isnan(quaternion.y) and math.isnan(quaternion.z) and math.isnan(quaternion.w)):
                self._log_error('Invalid orientation: {}'.format([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
                return
            parse_quaternion = False

        # convert message to expected format
        position = [position.x, position.y, position.z]
        if parse_quaternion:
            orientation = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_euler("xyz", degrees=False)
            orientation = [orientation[2], orientation[1], orientation[0]]  # alpha (z), beta (y), gamma (z)
        else:
            orientation = [math.nan] * 3

        # move the robot
        try:
            status = self._interface.command_cartesian_pose(position, orientation)
        except Exception as e:
            self._log_error('Failed to command cartesian pose to {}, {}'.format(position, orientation))
            self._log_error(str(e))
            return

        if not status:
            self._log_error('Failed to command cartesian pose to {}, {}'.format(position, orientation))
            self._log_error(str(self._interface.get_last_error()))
        if self._verbose:
            self._log_info('Commanded cartesian pose to {}, {} ({})'.format(position, orientation, status))

    # configuration commands (limits)

    def _handler_set_desired_joint_velocity_rel(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_joint_velocity_rel(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_joint_velocity_rel to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_joint_velocity_rel to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_joint_velocity_rel to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_desired_joint_acceleration_rel(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_joint_acceleration_rel(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_joint_acceleration_rel to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_joint_acceleration_rel to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_joint_acceleration_rel to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_desired_joint_jerk_rel(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_joint_jerk_rel(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_joint_jerk_rel to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_joint_jerk_rel to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_joint_jerk_rel to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_desired_cartesian_velocity(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_cartesian_velocity(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_cartesian_velocity to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_cartesian_velocity to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_cartesian_velocity to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_desired_cartesian_acceleration(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_cartesian_acceleration(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_cartesian_acceleration to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_cartesian_acceleration to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_cartesian_acceleration to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_desired_cartesian_jerk(self, request, response):  # DONE
        try:
            response.success = self._interface.set_desired_cartesian_jerk(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_desired_cartesian_jerk to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_desired_cartesian_jerk to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_desired_cartesian_jerk to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_cartesian_additional_control_force(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        try:
            response.success = self._interface.set_cartesian_additional_control_force(translational, rotational)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_additional_control_force to {}{}'.format(translational, rotational))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_additional_control_force to {}{}'.format(translational, rotational))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_additional_control_force to {}{} ({}, {})" \
                .format(translational, rotational, response.success, response.message))
        return response

    def _handler_set_cartesian_max_velocity(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        try:
            response.success = self._interface.set_cartesian_max_velocity(translational, rotational)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_max_velocity to {}{}'.format(translational, rotational))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_max_velocity to {}{}'.format(translational, rotational))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_max_velocity to {}{} ({}, {})" \
                .format(translational, rotational, response.success, response.message))
        return response

    def _handler_set_cartesian_max_path_deviation(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        try:
            response.success = self._interface.set_cartesian_max_path_deviation(translational, rotational)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_max_path_deviation to {}{}'.format(translational, rotational))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_max_path_deviation to {}{}'.format(translational, rotational))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_max_path_deviation to {}{} ({}, {})" \
                .format(translational, rotational, response.success, response.message))
        return response

    def _handler_set_cartesian_stiffness(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        null_space = request.float_param
        try:
            response.success = self._interface.set_cartesian_stiffness(translational, rotational, null_space)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_stiffness to {}{}({})'.format(translational, rotational, null_space))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_stiffness to {}{}({})'.format(translational, rotational, null_space))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_stiffness to {}{}({}) ({}, {})" \
                .format(translational, rotational, null_space, response.success, response.message))
        return response

    def _handler_set_cartesian_damping(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        null_space = request.float_param
        try:
            response.success = self._interface.set_cartesian_damping(translational, rotational, null_space)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_damping to {}{}({})'.format(translational, rotational, null_space))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_damping to {}{}({})'.format(translational, rotational, null_space))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_damping to {}{}({}) ({}, {})" \
                .format(translational, rotational, null_space, response.success, response.message))
        return response

    def _handler_set_cartesian_max_control_force(self, request, response):
        translational = [request.x, request.y, request.z]
        rotational = [request.a, request.b, request.c]
        add_stop_condition = request.boolean_param
        try:
            response.success = self._interface.set_cartesian_max_control_force(translational, rotational, add_stop_condition)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_cartesian_max_control_force to {}{}({})'.format(translational, rotational, add_stop_condition))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_cartesian_max_control_force to {}{}({})'.format(translational, rotational, add_stop_condition))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_cartesian_max_control_force to {}{}({}) ({}, {})" \
                .format(translational, rotational, add_stop_condition, response.success, response.message))
        return response

    def _handler_set_joint_stiffness(self, request, response):
        try:
            response.success = self._interface.set_joint_stiffness(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_joint_stiffness to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_joint_stiffness to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_joint_stiffness to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_joint_damping(self, request, response):
        try:
            response.success = self._interface.set_joint_damping(request.data)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_joint_damping to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_joint_damping to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_joint_damping to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    # conditions

    def _handler_reset_conditions(self, request, response):
        try:
            status = self._interface.reset_conditions()
            if not status:
                message = str(self._interface.get_last_error())
                self._log_error('Failed to reset_conditions')
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            self._log_error('Failed to reset_conditions')
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service reset_conditions")
        return response

    def _handler_set_force_condition(self, request, response):
        threshold = request.data[:3]
        tolerance = request.data[3:] if len(request.data) == 6 else [10, 10, 10]
        try:
            response.success = self._interface.set_force_condition(threshold, tolerance)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_force_condition to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_force_condition to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_force_condition to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_joint_torque_condition(self, request, response):
        lower_limits = request.data[:7]
        upper_limits = request.data[7:]
        try:
            response.success = self._interface.set_joint_torque_condition(lower_limits, upper_limits)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_joint_torque_condition to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_joint_torque_condition to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_joint_torque_condition to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    # configuration commands (motion and control)

    def _handler_set_control_interface(self, request, response) -> None:  # DONE
        control_interface = {"standard": libiiwa.ControlInterface.CONTROL_INTERFACE_STANDARD,
                             "servo": libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO}
        control_interface = control_interface.get(request.data.lower(), None)
        if control_interface is None:
            response.success = False
            response.message = "Unknown control interface: {}".format(request.data)
            self._log_error("Unknown control interface: {}".format(request.data))
            if self._verbose:
                self._log_info("Service set_control_interface to {} ({}, {})" \
                    .format(request.data, response.success, response.message))
            return response
        try:
            response.success = self._interface.set_control_interface(control_interface)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_control_interface to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_control_interface to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_control_interface to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_motion_type(self, request, response) -> None:  # DONE
        motion_type = {"ptp": libiiwa.MotionType.MOTION_TYPE_PTP,
                       "lin": libiiwa.MotionType.MOTION_TYPE_LIN,
                       "lin_rel": libiiwa.MotionType.MOTION_TYPE_LIN_REL,
                       "circ": libiiwa.MotionType.MOTION_TYPE_CIRC}
        motion_type = motion_type.get(request.data.lower(), None)
        if motion_type is None:
            response.success = False
            response.message = "Unknown motion type: {}".format(request.data)
            self._log_error("Unknown motion type: {}".format(request.data))
            if self._verbose:
                self._log_info("Service set_motion_type to {} ({}, {})" \
                    .format(request.data, response.success, response.message))
            return response
        try:
            response.success = self._interface.set_motion_type(motion_type)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_motion_type to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_motion_type to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_motion_type to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_control_mode(self, request, response) -> None:  # DONE
        control_mode = {"position": libiiwa.ControlMode.CONTROL_MODE_POSITION,
                        "joint_impedance": libiiwa.ControlMode.CONTROL_MODE_JOINT_IMPEDANCE,
                        "cartesian_impedance": libiiwa.ControlMode.CONTROL_MODE_CARTESIAN_IMPEDANCE,
                        "cartesian_sine_impedance": libiiwa.ControlMode.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE}
        control_mode = control_mode.get(request.data.lower(), None)
        if control_mode is None:
            response.success = False
            response.message = "Unknown control mode: {}".format(request.data)
            self._log_error("Unknown control mode: {}".format(request.data))
            if self._verbose:
                self._log_info("Service set_control_mode to {} ({}, {})" \
                    .format(request.data, response.success, response.message))
            return response
        try:
            response.success = self._interface.set_control_mode(control_mode)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_control_mode to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_control_mode to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_control_mode to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_execution_type(self, request, response) -> None:  # DONE
        execution_type = {"asynchronous": libiiwa.ExecutionType.EXECUTION_TYPE_ASYNCHRONOUS,
                          "synchronous": libiiwa.ExecutionType.EXECUTION_TYPE_SYNCHRONOUS}
        execution_type = execution_type.get(request.data.lower(), None)
        if execution_type is None:
            response.success = False
            response.message = "Unknown execution type: {}".format(request.data)
            self._log_error("Unknown execution type: {}".format(request.data))
            if self._verbose:
                self._log_info("Service set_execution_type to {} ({}, {})" \
                    .format(request.data, response.success, response.message))
            return response
        try:
            response.success = self._interface.set_execution_type(execution_type)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_execution_type to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_execution_type to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_execution_type to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_set_communication_mode(self, request, response) -> None:  # DONE
        communication_mode = {"on-demand": libiiwa.CommunicationMode.COMMUNICATION_MODE_ON_DEMAND,
                              "periodical": libiiwa.CommunicationMode.COMMUNICATION_MODE_PERIODICAL}
        communication_mode = communication_mode.get(request.data.lower(), None)
        if communication_mode is None:
            response.success = False
            response.message = "Unknown communication mode: {}".format(request.data)
            self._log_error("Unknown communication mode: {}".format(request.data))
            if self._verbose:
                self._log_info("Service set_communication_mode to {} ({}, {})" \
                    .format(request.data, response.success, response.message))
            return response
        try:
            response.success = self._interface.set_communication_mode(communication_mode)
            if not response.success:
                response.message = str(self._interface.get_last_error())
                self._log_error('Failed to set_communication_mode to {}'.format(request.data))
                self._log_error(str(self._interface.get_last_error()))
        except Exception as e:
            response.success = False
            response.message = str(e)
            self._log_error('Failed to set_communication_mode to {}'.format(request.data))
            self._log_error(str(e))
        if self._verbose:
            self._log_info("Service set_communication_mode to {} ({}, {})" \
                .format(request.data, response.success, response.message))
        return response

    def _handler_last_error(self, request, response):
        error = self._interface.get_last_error()
        response.error_code = error.value
        if self._verbose:
            self._log_info("Service last_error ({})".format(str(error)))
        return response

    def _handler_has_fired_condition(self, request, response):
        data = self._interface.get_state()["has_fired_condition"]
        response.data = data
        if self._verbose:
            self._log_info("Service has_fired_condition ({})".format(str(data)))
        return response
        
    def _handler_is_ready_to_move(self, request, response):
        data = self._interface.get_state()["is_ready_to_move"]
        response.data = data
        if self._verbose:
            self._log_info("Service is_ready_to_move ({})".format(str(data)))
        return response
        
    def _handler_has_active_motion(self, request, response):
        data = self._interface.get_state()["has_active_motion"]
        response.data = data
        if self._verbose:
            self._log_info("Service has_active_motion ({})".format(str(data)))
        return response

    def start(self) -> None:
        """Start the publisher
        """
        self.stop()

        # create publishers
        self._pub_joint_states = self._node.create_publisher(topic=self._names.get("joint_states", "/iiwa/state/joint_states"),
                                                             msg_type=sensor_msgs.msg.JointState,qos_profile=self._qos_profile)
        self._pub_end_effector_pose = self._node.create_publisher(topic=self._names.get("end_effector_pose", "/iiwa/state/end_effector_pose"),
                                                                  msg_type=geometry_msgs.msg.Pose,
                                                                  qos_profile=self._qos_profile)
        self._pub_end_effector_wrench = self._node.create_publisher(topic=self._names.get("end_effector_wrench", "/iiwa/state/end_effector_wrench"),
                                                                    msg_type=geometry_msgs.msg.Wrench,
                                                                    qos_profile=self._qos_profile)

        self._publishers = [self._pub_joint_states,
                            self._pub_end_effector_pose,
                            self._pub_end_effector_wrench]

        # create subscribers
        self._sub_stop_command = self._node.create_subscription(topic=self._names.get("stop_command", "/iiwa/command/stop"),
                                                                msg_type=std_msgs.msg.Empty,
                                                                callback=self._callback_stop_command,
                                                                qos_profile=self._qos_profile)
        self._sub_joint_command = self._node.create_subscription(topic=self._names.get("joint_command", "/iiwa/command/joint"),
                                                                 msg_type=sensor_msgs.msg.JointState,
                                                                 callback=self._callback_joint_command,
                                                                 qos_profile=self._qos_profile)
        self._sub_cartesian_command = self._node.create_subscription(topic=self._names.get("cartesian_command", "/iiwa/command/cartesian"),
                                                                     msg_type=geometry_msgs.msg.Pose,
                                                                     callback=self._callback_cartesian_command,
                                                                     qos_profile=self._qos_profile)

        self._subscribers = [self._sub_stop_command,
                             self._sub_joint_command,
                             self._sub_cartesian_command]

        # create services
        name = self._names.get("set_desired_joint_velocity_rel", "/iiwa/set_desired_joint_velocity_rel")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_joint_velocity_rel))

        name = self._names.get("set_desired_joint_acceleration_rel", "/iiwa/set_desired_joint_acceleration_rel")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_joint_acceleration_rel))

        name = self._names.get("set_desired_joint_jerk_rel", "/iiwa/set_desired_joint_jerk_rel")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_joint_jerk_rel))

        name = self._names.get("set_desired_cartesian_velocity", "/iiwa/set_desired_cartesian_velocity")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_cartesian_velocity))

        name = self._names.get("set_desired_cartesian_acceleration", "/iiwa/set_desired_cartesian_acceleration")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_cartesian_acceleration))

        name = self._names.get("set_desired_cartesian_jerk", "/iiwa/set_desired_cartesian_jerk")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetNumber,
                                                        callback=self._handler_set_desired_cartesian_jerk))

        name = self._names.get("set_cartesian_additional_control_force", "/iiwa/set_cartesian_additional_control_force")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABC,
                                                        callback=self._handler_set_cartesian_additional_control_force))

        name = self._names.get("set_cartesian_max_velocity", "/iiwa/set_cartesian_max_velocity")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABC,
                                                        callback=self._handler_set_cartesian_max_velocity))

        name = self._names.get("set_cartesian_max_path_deviation", "/iiwa/set_cartesian_max_path_deviation")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABC,
                                                        callback=self._handler_set_cartesian_max_path_deviation))

        name = self._names.get("set_cartesian_stiffness", "/iiwa/set_cartesian_stiffness")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABCParam,
                                                        callback=self._handler_set_cartesian_stiffness))

        name = self._names.get("set_cartesian_damping", "/iiwa/set_cartesian_damping")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABCParam,
                                                        callback=self._handler_set_cartesian_damping))

        name = self._names.get("set_cartesian_max_control_force", "/iiwa/set_cartesian_max_control_force")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetXYZABCParam,
                                                        callback=self._handler_set_cartesian_max_control_force))

        name = self._names.get("set_joint_stiffness", "/iiwa/set_joint_stiffness")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetArray,
                                                        callback=self._handler_set_joint_stiffness))

        name = self._names.get("set_joint_damping", "/iiwa/set_joint_damping")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetArray,
                                                        callback=self._handler_set_joint_damping))

        name = self._names.get("reset_conditions", "/iiwa/reset_conditions")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=Empty,
                                                        callback=self._handler_reset_conditions))

        name = self._names.get("set_force_condition", "/iiwa/set_force_condition")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetArray,
                                                        callback=self._handler_set_force_condition))

        name = self._names.get("set_joint_torque_condition", "/iiwa/set_joint_torque_condition")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetArray,
                                                        callback=self._handler_set_joint_torque_condition))

        name = self._names.get("set_control_interface", "/iiwa/set_control_interface")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetString,
                                                        callback=self._handler_set_control_interface))

        name = self._names.get("set_motion_type", "/iiwa/set_motion_type")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetString,
                                                        callback=self._handler_set_motion_type))

        name = self._names.get("set_control_mode", "/iiwa/set_control_mode")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetString,
                                                        callback=self._handler_set_control_mode))

        name = self._names.get("set_execution_type", "/iiwa/set_execution_type")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetString,
                                                        callback=self._handler_set_execution_type))

        name = self._names.get("set_communication_mode", "/iiwa/set_communication_mode")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=SetString,
                                                        callback=self._handler_set_communication_mode))

        name = self._names.get("last_error", "/iiwa/last_error")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=GetError,
                                                        callback=self._handler_last_error))

        name = self._names.get("has_fired_condition", "/iiwa/has_fired_condition")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=GetBool,
                                                        callback=self._handler_has_fired_condition))

        name = self._names.get("is_ready_to_move", "/iiwa/is_ready_to_move")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=GetBool,
                                                        callback=self._handler_is_ready_to_move))

        name = self._names.get("has_active_motion", "/iiwa/has_active_motion")
        self._services.append(self._node.create_service(srv_name=name,
                                                        srv_type=GetBool,
                                                        callback=self._handler_has_active_motion))

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

        orientation = state["cartesian_orientation"]  # alpha (z), beta (y), gamma (x)
        quaternion = Rotation.from_euler('xyz', [orientation[2], orientation[1], orientation[0]], degrees=False).as_quat()  # xyzw
        self._msg_end_effector_pose.orientation.x = quaternion[0].item()
        self._msg_end_effector_pose.orientation.y = quaternion[1].item()
        self._msg_end_effector_pose.orientation.z = quaternion[2].item()
        self._msg_end_effector_pose.orientation.w = quaternion[3].item()

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
    follow_all_trajectory = node.get_parameter('follow_all_trajectory').get_parameter_value().bool_value
    trajectory_update_threshold = node.get_parameter('trajectory_update_threshold').get_parameter_value().double_value

    libiiwa_ip = node.get_parameter('libiiwa_ip').get_parameter_value().string_value
    libiiwa_port = node.get_parameter('libiiwa_port').get_parameter_value().integer_value

    servo_interface = node.get_parameter('servo_interface').get_parameter_value().bool_value

    run_without_communication = node.get_parameter('run_without_communication').get_parameter_value().bool_value
    verbose = node.get_parameter('verbose').get_parameter_value().bool_value

    # init robot interface
    robot = LibIiwa(ip=libiiwa_ip, port=libiiwa_port, run_without_communication=run_without_communication)

    if servo_interface:
        robot.set_control_interface(libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO)
    else:
        robot.set_control_interface(libiiwa.ControlInterface.CONTROL_INTERFACE_STANDARD)

    robot.set_desired_joint_velocity_rel(0.5)
    robot.set_desired_joint_acceleration_rel(0.5)
    robot.set_desired_joint_jerk_rel(0.5)

    # load controllers
    controllers = [Iiwa(node=node,
                        interface=robot, 
                        joints=JOINTS, 
                        verbose=verbose),
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
            try:
                rate.sleep()
            except ROSInterruptException:
                break

        for controller in controllers:
            controller.stop()

    # start control loop
    import threading
    threading.Thread(target=control_loop, args=(node,)).start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # stop controllers
    for controller in controllers:
        controller.stop()

    node.destroy_node()
    rclpy.shutdown()
