#!/usr/bin/env python3
from typing import Optional, Mapping

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from libiiwa_msgs.srv import SetArray, SetArrayRequest, SetArrayResponse
from libiiwa_msgs.srv import SetNumber, SetNumberRequest, SetNumberResponse
from libiiwa_msgs.srv import SetString, SetStringRequest, SetStringResponse
from libiiwa_msgs.srv import SetXYZABC, SetXYZABCRequest, SetXYZABCResponse
from libiiwa_msgs.srv import SetXYZABCParam, SetXYZABCParamRequest, SetXYZABCParamResponse
from libiiwa_msgs.srv import GetError, GetErrorRequest, GetErrorResponse
from libiiwa_msgs.srv import GetBool, GetBoolRequest, GetBoolResponse
from libiiwa_msgs.srv import GetNumber, GetNumberRequest, GetNumberResponse

try:
    import libiiwa
except ModuleNotFoundError:
    import os
    import sys
    # get path relative to repository root
    libiiwa_path = os.path.abspath(os.path.join(os.path.dirname(__file__), *(['..'] * 4), 'libiiwa'))
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
                 interface: libiiwa.LibIiwa, 
                 joints: Mapping[str, dict], 
                 names: Optional[Mapping[str, str]] = {}, 
                 queue_size: int = 10,
                 verbose: bool = False) -> None:
        """ROS control interface for the KUKA LBR iiwa robot

        :param interface: libiiwa.LibIiwa object
        :type interface: libiiwa.LibIiwa
        :param joints: Joint names and parameters
        :type joints: Mapping[str, dict]
        :param names: Topics and services mapping to the corresponding ROS names (default: {})
        :type names: Mapping[str, str], optional
        :param queue_size: Size of the ROS publisher and subscriber queues (default: 10)
        :type queue_size: int, optional
        :param verbose: Print additional information (default: False)
        :type verbose: bool, optional
        """
        self._names = names
        self._joints = joints
        self._interface = interface
        self._queue_size = queue_size
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
        rospy.loginfo(msg)
    
    def _log_warn(self, msg):
        rospy.logwarn(msg)
    
    def _log_error(self, msg):
        rospy.logerr(msg)

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

    def _handler_set_desired_joint_velocity_rel(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_desired_joint_acceleration_rel(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_desired_joint_jerk_rel(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_desired_cartesian_velocity(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_desired_cartesian_acceleration(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_desired_cartesian_jerk(self, request: SetNumberRequest) -> None:  # DONE
        response = SetNumberResponse()
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

    def _handler_set_cartesian_additional_control_force(self, request: SetXYZABCRequest) -> None:
        response = SetXYZABCResponse()
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

    def _handler_set_cartesian_max_velocity(self, request: SetXYZABCRequest) -> None:
        response = SetXYZABCResponse()
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

    def _handler_set_cartesian_max_path_deviation(self, request: SetXYZABCRequest) -> None:
        response = SetXYZABCResponse()
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

    def _handler_set_cartesian_stiffness(self, request: SetXYZABCParamRequest):
        response = SetXYZABCParamResponse()
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

    def _handler_set_cartesian_damping(self, request: SetXYZABCParamRequest):
        response = SetXYZABCParamResponse()
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

    def _handler_set_cartesian_max_control_force(self, request: SetXYZABCParamRequest):
        response = SetXYZABCParamResponse()
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

    def _handler_set_joint_stiffness(self, request: SetArrayRequest):
        response = SetArrayResponse()
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

    def _handler_set_joint_damping(self, request: SetArrayRequest):
        response = SetArrayResponse()
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

    def _handler_reset_conditions(self, request: EmptyRequest):
        response = EmptyResponse()
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

    def _handler_set_force_condition(self, request: SetArrayRequest):
        response = SetArrayResponse()
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

    def _handler_set_joint_torque_condition(self, request: SetArrayRequest):
        response = SetArrayResponse()
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

    def _handler_set_control_interface(self, request: SetStringRequest) -> None:  # DONE
        response = SetStringResponse()
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

    def _handler_set_motion_type(self, request: SetStringRequest) -> None:  # DONE
        response = SetStringResponse()
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

    def _handler_set_control_mode(self, request: SetStringRequest) -> None:  # DONE
        response = SetStringResponse()
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

    def _handler_set_execution_type(self, request: SetStringRequest) -> None:  # DONE
        response = SetStringResponse()
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

    def _handler_set_communication_mode(self, request: SetStringRequest) -> None:  # DONE
        response = SetStringResponse()
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

    def _handler_last_error(self, request: GetErrorRequest) -> None:
        response = GetErrorResponse()
        error = self._interface.get_last_error()
        response.error_code = error.value
        if self._verbose:
            self._log_info("Service last_error ({})".format(str(error)))
        return response

    def _handler_has_fired_condition(self, request: GetBoolRequest) -> None:
        response = GetBoolResponse()
        data = self._interface.get_state()["has_fired_condition"]
        response.data = data
        if self._verbose:
            self._log_info("Service has_fired_condition ({})".format(str(data)))
        return response
        
    def _handler_is_ready_to_move(self, request: GetBoolRequest) -> None:
        response = GetBoolResponse()
        data = self._interface.get_state()["is_ready_to_move"]
        response.data = data
        if self._verbose:
            self._log_info("Service is_ready_to_move ({})".format(str(data)))
        return response
        
    def _handler_has_active_motion(self, request: GetBoolRequest) -> None:
        response = GetBoolResponse()
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
        self._pub_joint_states = rospy.Publisher(name=self._names.get("joint_states", "/iiwa/state/joint_states"),
                                                 data_class=sensor_msgs.msg.JointState,
                                                 queue_size=self._queue_size)
        self._pub_end_effector_pose = rospy.Publisher(name=self._names.get("end_effector_pose", "/iiwa/state/end_effector_pose"),
                                                      data_class=geometry_msgs.msg.Pose,
                                                      queue_size=self._queue_size)
        self._pub_end_effector_wrench = rospy.Publisher(name=self._names.get("end_effector_wrench", "/iiwa/state/end_effector_wrench"),
                                                        data_class=geometry_msgs.msg.Wrench,
                                                        queue_size=self._queue_size)

        self._publishers = [self._pub_joint_states,
                            self._pub_end_effector_pose,
                            self._pub_end_effector_wrench]

        # create subscribers
        self._sub_stop_command = rospy.Subscriber(name=self._names.get("stop_command", "/iiwa/command/stop"),
                                                  data_class=std_msgs.msg.Empty,
                                                  callback=self._callback_stop_command)
        self._sub_joint_command = rospy.Subscriber(name=self._names.get("joint_command", "/iiwa/command/joint"),
                                                   data_class=sensor_msgs.msg.JointState,
                                                   callback=self._callback_joint_command)
        self._sub_cartesian_command = rospy.Subscriber(name=self._names.get("cartesian_command", "/iiwa/command/cartesian"),
                                                       data_class=geometry_msgs.msg.Pose,
                                                       callback=self._callback_cartesian_command)

        self._subscribers = [self._sub_stop_command,
                             self._sub_joint_command,
                             self._sub_cartesian_command]

        # create services
        name = self._names.get("set_desired_joint_velocity_rel", "/iiwa/set_desired_joint_velocity_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_joint_velocity_rel))

        name = self._names.get("set_desired_joint_acceleration_rel", "/iiwa/set_desired_joint_acceleration_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_joint_acceleration_rel))

        name = self._names.get("set_desired_joint_jerk_rel", "/iiwa/set_desired_joint_jerk_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_joint_jerk_rel))

        name = self._names.get("set_desired_cartesian_velocity", "/iiwa/set_desired_cartesian_velocity")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_cartesian_velocity))

        name = self._names.get("set_desired_cartesian_acceleration", "/iiwa/set_desired_cartesian_acceleration")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_cartesian_acceleration))

        name = self._names.get("set_desired_cartesian_jerk", "/iiwa/set_desired_cartesian_jerk")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetNumber,
                                            handler=self._handler_set_desired_cartesian_jerk))

        name = self._names.get("set_cartesian_additional_control_force", "/iiwa/set_cartesian_additional_control_force")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABC,
                                            handler=self._handler_set_cartesian_additional_control_force))

        name = self._names.get("set_cartesian_max_velocity", "/iiwa/set_cartesian_max_velocity")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABC,
                                            handler=self._handler_set_cartesian_max_velocity))

        name = self._names.get("set_cartesian_max_path_deviation", "/iiwa/set_cartesian_max_path_deviation")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABC,
                                            handler=self._handler_set_cartesian_max_path_deviation))

        name = self._names.get("set_cartesian_stiffness", "/iiwa/set_cartesian_stiffness")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABCParam,
                                            handler=self._handler_set_cartesian_stiffness))

        name = self._names.get("set_cartesian_damping", "/iiwa/set_cartesian_damping")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABCParam,
                                            handler=self._handler_set_cartesian_damping))

        name = self._names.get("set_cartesian_max_control_force", "/iiwa/set_cartesian_max_control_force")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetXYZABCParam,
                                            handler=self._handler_set_cartesian_max_control_force))

        name = self._names.get("set_joint_stiffness", "/iiwa/set_joint_stiffness")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetArray,
                                            handler=self._handler_set_joint_stiffness))

        name = self._names.get("set_joint_damping", "/iiwa/set_joint_damping")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetArray,
                                            handler=self._handler_set_joint_damping))

        name = self._names.get("reset_conditions", "/iiwa/reset_conditions")
        self._services.append(rospy.Service(name=name,
                                            service_class=Empty,
                                            handler=self._handler_reset_conditions))

        name = self._names.get("set_force_condition", "/iiwa/set_force_condition")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetArray,
                                            handler=self._handler_set_force_condition))

        name = self._names.get("set_joint_torque_condition", "/iiwa/set_joint_torque_condition")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetArray,
                                            handler=self._handler_set_joint_torque_condition))

        name = self._names.get("set_control_interface", "/iiwa/set_control_interface")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_control_interface))

        name = self._names.get("set_motion_type", "/iiwa/set_motion_type")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_motion_type))

        name = self._names.get("set_control_mode", "/iiwa/set_control_mode")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_control_mode))

        name = self._names.get("set_execution_type", "/iiwa/set_execution_type")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_execution_type))

        name = self._names.get("set_communication_mode", "/iiwa/set_communication_mode")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_communication_mode))

        name = self._names.get("last_error", "/iiwa/last_error")
        self._services.append(rospy.Service(name=name,
                                            service_class=GetError,
                                            handler=self._handler_last_error))

        name = self._names.get("has_fired_condition", "/iiwa/has_fired_condition")
        self._services.append(rospy.Service(name=name,
                                            service_class=GetBool,
                                            handler=self._handler_has_fired_condition))

        name = self._names.get("is_ready_to_move", "/iiwa/is_ready_to_move")
        self._services.append(rospy.Service(name=name,
                                            service_class=GetBool,
                                            handler=self._handler_is_ready_to_move))

        name = self._names.get("has_active_motion", "/iiwa/has_active_motion")
        self._services.append(rospy.Service(name=name,
                                            service_class=GetBool,
                                            handler=self._handler_has_active_motion))

    def stop(self) -> None:
        """Stop the publisher
        """
        for publisher in self._publishers:
            publisher.unregister()
        for subscriber in self._subscribers:
            subscriber.unregister()

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


class FollowJointTrajectory:
    def __init__(self, 
                 interface: libiiwa.LibIiwa, 
                 action_name: str, 
                 joints: dict, 
                 follow_all_trajectory: bool = True, 
                 trajectory_update_threshold = 0.5,
                 verbose: bool = False):
        self._interface = interface
        self._joints = joints
        self._follow_all_trajectory = follow_all_trajectory
        self._trajectory_update_threshold = trajectory_update_threshold
        self._verbose = verbose

        self._action_name = action_name
        self._action_server = None

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = control_msgs.msg.FollowJointTrajectoryResult()
        self._action_feedback_message = control_msgs.msg.FollowJointTrajectoryFeedback()

    def start(self) -> None:
        """Start the action server
        """
        self.stop()
        self._action_server = actionlib.ActionServer(self._action_name,
                                                     control_msgs.msg.FollowJointTrajectoryAction,
                                                     goal_cb=self._on_goal,
                                                     cancel_cb=self._on_cancel,
                                                     auto_start=False)
        self._action_server.start()

    def stop(self) -> None:
        """Stop the action server
        """
        # noetic/lib/python3/dist-packages/actionlib/action_server.py
        if self._action_server:
            if self._action_server.started:
                self._action_server.started = False

                self._action_server.status_pub.unregister()
                self._action_server.result_pub.unregister()
                self._action_server.feedback_pub.unregister()

                self._action_server.goal_sub.unregister()
                self._action_server.cancel_sub.unregister()

            del self._action_server
            self._action_server = None

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float

        :return: The current position of the joint
        :rtype: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        return target_position

    def _get_joint_position(self, name: str, state: list) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param state: The joint states
        :type state: list

        :return: The current position of the joint
        :rtype: float
        """
        return state[self._joints[name]["index"]]

    def _on_goal(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling new goal requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        goal = goal_handle.get_goal()

        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                self._log_warn("FollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                self._action_result_message.error_code = self._action_result_message.INVALID_JOINTS
                self._action_result_message.error_string = "Joints don't match: {} not in {}".format(name, list(self._joints.keys()))
                goal_handle.set_rejected(self._action_result_message, "")
                if self._verbose:
                    self._log_info("FollowJointTrajectory: goal request rejected")
                return

        # reject if there is an active goal
        if self._action_goal is not None:
            self._log_warn("FollowJointTrajectory: multiple goals not supported")
            self._action_result_message.error_code = self._action_result_message.INVALID_GOAL
            self._action_result_message.error_string = "Multiple goals not supported"
            goal_handle.set_rejected(self._action_result_message, "")
            if self._verbose:
                self._log_info("FollowJointTrajectory: goal request rejected")
            return

        # use specified trajectory points
        if not self._follow_all_trajectory:
            goal.trajectory.points = [goal.trajectory.points[-1]]

        # check initial position
        if goal.trajectory.points[0].time_from_start.to_sec():
            state = self._interface.get_state()["joint_position"]
            initial_point = JointTrajectoryPoint(positions=[self._get_joint_position(name, state) \
                for name in goal.trajectory.joint_names], time_from_start=rospy.Duration())
            goal.trajectory.points.insert(0, initial_point)

        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_point_index = 0
        self._action_start_time = rospy.get_time()
        self._action_feedback_message.joint_names = list(
            goal.trajectory.joint_names)

        goal_handle.set_accepted()
        if self._verbose:
            self._log_info("FollowJointTrajectory: goal request accepted")

    def _on_cancel(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        if self._action_goal is None:
            goal_handle.set_rejected()
            if self._verbose:
                self._log_info("FollowJointTrajectory: cancel request rejected")
            return

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        
        # stop the robot
        self._interface.command_stop()
        
        goal_handle.set_canceled()
        if self._verbose:
            self._log_info("FollowJointTrajectory: cancel request accepted")

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        if self._action_goal is not None and self._action_goal_handle is not None:
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = rospy.get_time() - self._action_start_time

            state = self._interface.get_state()
            
            # check robot status
            should_abort = False
            if state["has_fired_condition"]:
                should_abort = True
                error_code = -6
                error_string = "A condition has been fired"
            elif not state["is_ready_to_move"]:
                should_abort = True
                error_code = -7
                error_string = "Robot is not ready to move"
            elif not state["has_active_motion"]:
                should_abort = True
                error_code = -8
                error_string = "Robot has not active motion"

            if should_abort:
                self._log_warn("FollowJointTrajectory: aborted execution. ")
                self._action_goal = None
                self._action_result_message.error_code = error_code
                self._action_result_message.error_string = error_string
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_aborted(self._action_result_message)
                    self._action_goal_handle = None
                return

            current_position = np.array([self._get_joint_position(name, state["joint_position"]) 
                                         for name in self._action_goal.trajectory.joint_names])
            
            diff = np.abs(np.array(current_point.positions) - current_position).sum()
            if diff <= self._trajectory_update_threshold:
                self._action_point_index += 1
                # end of trajectory
                if self._action_point_index >= len(self._action_goal.trajectory.points):
                    if self._verbose:
                        self._log_info("FollowJointTrajectory: set succeeded trajectory")
                    self._action_goal = None
                    self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                    self._action_result_message.error_string = ""
                    if self._action_goal_handle is not None:
                        self._action_goal_handle.set_succeeded(self._action_result_message)
                        self._action_goal_handle = None
                    return
                # update target
                if self._verbose:
                    self._log_info("FollowJointTrajectory: update trajectory {}/{} points ({})" \
                        .format(self._action_point_index, len(self._action_goal.trajectory.points), diff))
                current_point = self._action_goal.trajectory.points[self._action_point_index]
                joint_positions = [0] * len(self._action_goal.trajectory.joint_names)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    joint_positions[i] = self._set_joint_position(name, current_point.positions[i])
                self._interface.command_joint_position(joint_positions)
                # send feedback
                if self._verbose:
                    self._log_info("FollowJointTrajectory: send feedback after {} seconds".format(time_passed))
                self._action_feedback_message.actual.positions = [self._get_joint_position(name, state)
                                                                  for name in self._action_goal.trajectory.joint_names]
                self._action_feedback_message.actual.time_from_start = rospy.Duration.from_sec(time_passed)
                if self._action_goal_handle is not None:
                    self._action_goal_handle.publish_feedback(self._action_feedback_message)




def main():

    from libiiwa import LibIiwa

    # init node
    rospy.init_node("iiwa")
    rate = rospy.Rate(50)  # Hz

    # get launch parameters
    robot_name = rospy.get_param("~robot_name", "iiwa")  # TODO: use as prefix

    controller_name = rospy.get_param("~controller_name", "iiwa_controller")
    action_namespace = rospy.get_param("~action_namespace", "follow_joint_trajectory")
    follow_all_trajectory = rospy.get_param("~follow_all_trajectory", True)
    trajectory_update_threshold = rospy.get_param("~trajectory_update_threshold", 0.5)

    libiiwa_ip = rospy.get_param("~libiiwa_ip", "0.0.0.0")
    libiiwa_port = rospy.get_param("~libiiwa_port", 12225)
    
    servo_interface = rospy.get_param("~servo_interface", True)

    run_without_communication = rospy.get_param("~run_without_communication", False)
    verbose = rospy.get_param("~verbose", False)

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
    controllers = [Iiwa(interface=robot, 
                        joints=JOINTS, 
                        verbose=verbose),
                   FollowJointTrajectory(interface=robot, 
                                         action_name=f"/{controller_name}/{action_namespace}", 
                                         joints=JOINTS, 
                                         follow_all_trajectory=follow_all_trajectory, 
                                         trajectory_update_threshold=trajectory_update_threshold,
                                         verbose=verbose)]

    # control loop
    for controller in controllers:
        controller.start()

    while not rospy.is_shutdown():
        robot.get_state(refresh=True)
        for controller in controllers:
            controller.step(0)
        rate.sleep()

    for controller in controllers:
        controller.stop()




if __name__ == "__main__":
    main()
