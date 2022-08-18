#!/usr/bin/env python3

import rospy
import actionlib
import sensor_msgs.msg
import geometry_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint

from libiiwa_msgs.srv import SetDouble, SetDoubleResponse
from libiiwa_msgs.srv import SetString, SetStringResponse

try:
    import libiiwa
except ModuleNotFoundError:
    import os
    import sys
    # get path relative to repository root
    libiiwa_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'libiiwa'))
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
    def __init__(self, robot, joints: dict, names: dict = {}, queue_size: int = 10):
        self._robot = robot
        self._joints = joints

        self._names = names
        self._queue_size = queue_size

        # publishers
        self._publishers = []
        self._pub_joint_states = None
        self._pub_end_effector_pose = None
        self._pub_end_effector_wrench = None

        # services
        self._services = []
        self._srv_set_desired_joint_velocity_rel = None

        # messages
        self._msg_joint_states = sensor_msgs.msg.JointState()
        self._msg_end_effector_pose = geometry_msgs.msg.Pose()
        self._msg_end_effector_wrench = geometry_msgs.msg.Wrench()

        # initialize messages
        self._msg_joint_states.name = sorted(list(self._joints.keys()))

    def _handler_set_desired_joint_velocity_rel(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_joint_velocity_rel(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_desired_joint_acceleration_rel(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_joint_acceleration_rel(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_desired_joint_jerk_rel(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_joint_jerk_rel(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_desired_cartesian_velocity(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_cartesian_velocity(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_desired_cartesian_acceleration(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_cartesian_acceleration(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_desired_cartesian_jerk(self, request):
        response = SetDoubleResponse()
        try:
            response.success = self._robot.set_desired_cartesian_jerk(request.data)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_control_interface(self, request):
        response = SetStringResponse()
        control_interface = {"standard": libiiwa.ControlInterface.CONTROL_INTERFACE_STANDARD,
                             "servo": libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO}
        control_interface = control_interface.get(request.data, None)
        if control_interface is None:
            response.success = False
            response.message = "Unknown control interface: {}".format(request.data)
            return response
        try:
            response.success = self._robot.set_control_interface(control_interface)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_motion_type(self, request):
        response = SetStringResponse()
        motion_type = {"ptp": libiiwa.MotionType.MOTION_TYPE_PTP,
                       "lin": libiiwa.MotionType.MOTION_TYPE_LIN,
                       "lin_rel": libiiwa.MotionType.MOTION_TYPE_LIN_REL,
                       "circ": libiiwa.MotionType.MOTION_TYPE_CIRC}
        motion_type = motion_type.get(request.data, None)
        if motion_type is None:
            response.success = False
            response.message = "Unknown motion type: {}".format(request.data)
            return response
        try:
            response.success = self._robot.set_motion_type(motion_type)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_control_mode(self, request):
        response = SetStringResponse()
        control_mode = {"position": libiiwa.ControlMode.CONTROL_MODE_POSITION,
                        "joint_impedance": libiiwa.ControlMode.CONTROL_MODE_JOINT_IMPEDANCE,
                        "cartesian_impedance": libiiwa.ControlMode.CONTROL_MODE_CARTESIAN_IMPEDANCE,
                        "cartesian_sine_impedance": libiiwa.ControlMode.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE}
        control_mode = control_mode.get(request.data, None)
        if control_mode is None:
            response.success = False
            response.message = "Unknown control mode: {}".format(request.data)
            return response
        try:
            response.success = self._robot.set_control_mode(control_mode)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handler_set_communication_mode(self, request):
        response = SetStringResponse()
        communication_mode = {"on-demand": libiiwa.CommunicationMode.COMMUNICATION_MODE_ON_DEMAND,
                              "periodical": libiiwa.CommunicationMode.COMMUNICATION_MODE_PERIODICAL}
        communication_mode = communication_mode.get(request.data, None)
        if communication_mode is None:
            response.success = False
            response.message = "Unknown communication mode: {}".format(request.data)
            return response
        try:
            response.success = self._robot.set_communication_mode(communication_mode)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def start(self) -> None:
        """Start the publisher
        """
        self.stop()

        # create publishers
        self._pub_joint_states = rospy.Publisher(self._names.get("joint_states", "/iiwa/joint_states"),
                                                 sensor_msgs.msg.JointState,
                                                 queue_size=self._queue_size)
        self._pub_end_effector_pose = rospy.Publisher(self._names.get("end_effector_pose", "/iiwa/end_effector_pose"),
                                                      geometry_msgs.msg.Pose,
                                                      queue_size=self._queue_size)
        self._pub_end_effector_wrench = rospy.Publisher(self._names.get("end_effector_wrench", "/iiwa/end_effector_wrench"),
                                                        geometry_msgs.msg.Wrench,
                                                        queue_size=self._queue_size)

        self._publishers = [self._pub_joint_states,
                            self._pub_end_effector_pose,
                            self._pub_end_effector_wrench]

        # create services
        name = self._names.get("set_desired_joint_velocity_rel", "/iiwa/set_desired_joint_velocity_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_joint_velocity_rel))

        name = self._names.get("set_desired_joint_acceleration_rel", "/iiwa/set_desired_joint_acceleration_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_joint_acceleration_rel))

        name = self._names.get("set_desired_joint_jerk_rel", "/iiwa/set_desired_joint_jerk_rel")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_joint_jerk_rel))

        name = self._names.get("set_desired_cartesian_velocity", "/iiwa/set_desired_cartesian_velocity")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_cartesian_velocity))

        name = self._names.get("set_desired_cartesian_acceleration", "/iiwa/set_desired_cartesian_acceleration")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_cartesian_acceleration))

        name = self._names.get("set_desired_cartesian_jerk", "/iiwa/set_desired_cartesian_jerk")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetDouble,
                                            handler=self._handler_set_desired_cartesian_jerk))

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

        name = self._names.get("set_communication_mode", "/iiwa/set_communication_mode")
        self._services.append(rospy.Service(name=name,
                                            service_class=SetString,
                                            handler=self._handler_set_communication_mode))

    def stop(self) -> None:
        """Stop the publisher
        """
        for publisher in self._publishers:
            publisher.unregister()

        self._pub_joint_states = None
        self._pub_end_effector_pose = None
        self._pub_end_effector_wrench = None

        self._publishers = []
        self._services = []

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        state = self._robot.get_state()

        # TODO: set header

        # joint states
        self._msg_joint_states.position = state["joint_position"]
        self._msg_joint_states.velocity = state["joint_velocity"]
        self._msg_joint_states.effort = state["joint_torque"]

        # end-effector pose
        position = state["cartesian_position"]
        self._msg_end_effector_pose.position.x = position[0]
        self._msg_end_effector_pose.position.y = position[1]
        self._msg_end_effector_pose.position.z = position[2]

        # TODO: convert to quaternion
        orientation = state["cartesian_orientation"]
        self._msg_end_effector_pose.orientation.x = orientation[0]
        self._msg_end_effector_pose.orientation.y = orientation[1]
        self._msg_end_effector_pose.orientation.z = orientation[2]
        self._msg_end_effector_pose.orientation.w = orientation[0]

        # end-effector wrench
        force = state["cartesian_force"]
        self._msg_end_effector_wrench.force.x = force[0]
        self._msg_end_effector_wrench.force.y = force[1]
        self._msg_end_effector_wrench.force.z = force[2]

        torque = state["cartesian_torque"]
        self._msg_end_effector_wrench.torque.x = torque[0]
        self._msg_end_effector_wrench.torque.y = torque[1]
        self._msg_end_effector_wrench.torque.z = torque[2]

        # publish
        self._pub_joint_states.publish(self._msg_joint_states)
        self._pub_end_effector_pose.publish(self._msg_end_effector_pose)
        self._pub_end_effector_wrench.publish(self._msg_end_effector_wrench)


class FollowJointTrajectory:
    def __init__(self, robot, action_name: str, joints: dict):
        self._robot = robot
        self._joints = joints

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
        print("FollowJointTrajectory: destroying action server")
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
                print("[Warning]FollowJointTrajectory: joints don't match ({} not in {})"
                      .format(name, list(self._joints.keys())))
                self._action_result_message.error_code = self._action_result_message.INVALID_JOINTS
                goal_handle.set_rejected(self._action_result_message, "")
                return

        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning]FollowJointTrajectory: multiple goals not supported")
            self._action_result_message.error_code = self._action_result_message.INVALID_GOAL
            goal_handle.set_rejected(self._action_result_message, "")
            return

        # check initial position
        if goal.trajectory.points[0].time_from_start.to_sec():
            initial_point = JointTrajectoryPoint(positions=[self._get_joint_position(name) for name in goal.trajectory.joint_names],
                                                 time_from_start=rospy.Duration())
            goal.trajectory.points.insert(0, initial_point)

        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_point_index = 1
        self._action_start_time = rospy.get_time()
        self._action_feedback_message.joint_names = list(
            goal.trajectory.joint_names)

        goal_handle.set_accepted()

    def _on_cancel(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        if self._action_goal is None:
            goal_handle.set_rejected()
            return
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        goal_handle.set_canceled()

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        if self._action_goal is not None and self._action_goal_handle is not None:
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                self._action_goal = None
                self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(
                        self._action_result_message)
                    self._action_goal_handle = None
                return

            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = rospy.get_time() - self._action_start_time

            # set target using linear interpolation
            if time_passed <= current_point.time_from_start.to_sec():
                ratio = (time_passed - previous_point.time_from_start.to_sec()) \
                    / (current_point.time_from_start.to_sec() - previous_point.time_from_start.to_sec())
                joint_positions = [0] * \
                    len(self._action_goal.trajectory.joint_names)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = - \
                        1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                        + side * ratio * \
                        abs(current_point.positions[i] -
                            previous_point.positions[i])
                    joint_positions[i] = self._set_joint_position(
                        name, target_position)
                    self._robot.command_joint_position(joint_positions)
            # send feedback
            else:
                state = self._robot.get_state()["joint_position"]
                self._action_point_index += 1
                self._action_feedback_message.actual.positions = [self._get_joint_position(name, state)
                                                                  for name in self._action_goal.trajectory.joint_names]
                self._action_feedback_message.actual.time_from_start = rospy.Duration.from_sec(
                    time_passed)
                if self._action_goal_handle is not None:
                    self._action_goal_handle.publish_feedback(
                        self._action_feedback_message)




if __name__ == "__main__":

    from libiiwa import LibIiwa

    # init node
    rospy.init_node("iiwa")
    rate = rospy.Rate(50)  # Hz

    # get launch parameters
    robot_name = rospy.get_param("~robot_name", "iiwa")
    controller_name = rospy.get_param("~controller_name", "iiwa_controller")
    action_namespace = rospy.get_param("~action_namespace", "follow_joint_trajectory")
    libiiwa_ip = rospy.get_param("~libiiwa_ip", "0.0.0.0")
    libiiwa_port = rospy.get_param("~libiiwa_port", 12225)

    # init robot interface
    robot = LibIiwa(ip=libiiwa_ip, port=libiiwa_port)
    robot.start()

    robot.set_control_interface(libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO)
    robot.set_desired_joint_velocity_rel(0.5)

    controllers = [Iiwa(robot, JOINTS),
                   FollowJointTrajectory(robot, f"/{controller_name}/{action_namespace}", JOINTS)]

    for controller in controllers:
        controller.start()

    while not rospy.is_shutdown():
        robot.get_state(refresh=True)
        for controller in controllers:
            controller.step(0)
        rate.sleep()

    for controller in controllers:
        controller.stop()
