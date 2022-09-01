from typing import Optional, Union, List

import time
import struct
import socket
import threading
import collections
import numpy as np
from enum import Enum


__all__ = [
    'LibIiwa',
    'MotionType',
    'ControlMode',
    'ExecutionType',
    'ControlInterface',
    'Error',
    'API_VERSION',
]

__version__ = '0.1.0-beta'
API_VERSION = "0.1.0-beta"

# application errors
class Error(Enum):
    INVALID_CONFIGURATION_ERROR = -16
    VALIDATION_FOR_IMPEDANCE_ERROR = -15
    ASYNCHRONOUS_MOTION_ERROR = -14
    SYNCHRONOUS_MOTION_ERROR = -13
    INVALID_JOINT_ERROR = -12
    VALUE_ERROR = -11
    NO_ERROR = -10

# empty commands
COMMAND_PASS = 0

# communication modes
class CommunicationMode(Enum):
    COMMUNICATION_MODE_ON_DEMAND = 11
    COMMUNICATION_MODE_PERIODICAL = 12

# motion types
class MotionType(Enum):
    """Configurable motion types
    """
    MOTION_TYPE_PTP = 21
    MOTION_TYPE_LIN = 22
    MOTION_TYPE_LIN_REL = 23
    MOTION_TYPE_CIRC = 24

# control interfaces
class ControlInterface(Enum):
    """Configurable control interfaces
    """
    CONTROL_INTERFACE_STANDARD = 31
    CONTROL_INTERFACE_SERVO = 32
    CONTROL_INTERFACE_FRI = 33

# control modes
class ControlMode(Enum):
    """Configurable control modes
    """
    CONTROL_MODE_POSITION = 41
    CONTROL_MODE_JOINT_IMPEDANCE = 42
    CONTROL_MODE_CARTESIAN_IMPEDANCE = 43
    CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE = 44

# execution type
class ExecutionType(Enum):
    """Configurable execution types
    """
    EXECUTION_TYPE_ASYNCHRONOUS = 51
    EXECUTION_TYPE_SYNCHRONOUS = 52

# control command
COMMAND_JOINT_POSITION = 101
COMMAND_CARTESIAN_POSE = 102
COMMAND_CIRC_MOTION = 103

# configuration commands (limits)
COMMAND_SET_DESIRED_JOINT_VELOCITY_REL = 201
COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL = 202
COMMAND_SET_DESIRED_JOINT_JERK_REL = 203
COMMAND_SET_DESIRED_CARTESIAN_VELOCITY = 204
COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION = 205
COMMAND_SET_DESIRED_CARTESIAN_JERK = 206
# configuration commands (conditions)
COMMAND_SET_FORCE_CONDITION = 211
COMMAND_SET_JOINT_TORQUE_CONDITION = 212
# configuration commands (impedance control)
COMMAND_SET_CARTESIAN_STIFFNESS = 221
COMMAND_SET_CARTESIAN_DAMPING = 222
COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE = 223
COMMAND_SET_JOINT_STIFFNESS = 224
COMMAND_SET_JOINT_DAMPING = 225
# configuration commands (motion and control)
COMMAND_SET_COMMUNICATION_MODE = 301
COMMAND_SET_CONTROL_INTERFACE = 302
COMMAND_SET_MOTION_TYPE = 303
COMMAND_SET_CONTROL_MODE = 304
COMMAND_SET_EXECUTION_TYPE = 305


class LibIiwaCommunication:
    def __init__(self,
                 ip: Optional[str] = "0.0.0.0",
                 port: Optional[int] = 12225,
                 queue_size: Optional[int] = 10,
                 run_without_communication: Optional[bool] = False) -> None:
        """Library communication endpoint

        :param ip: IP address of the library communication endpoint (default: all interfaces)
        :type ip: str, optional
        :param port: port of the library communication endpoint (default: 12225)
        :type port: int, optional
        :param queue_size: size of the command queue (default: 10)
        :type queue_size: int, optional
        :param run_without_communication: Run the library without creating the communication socket (default: False).
                                          Useful for testing the library without the robot
        :type run_without_communication: bool, optional
        """
        self._running = False
        self._communication_mode = CommunicationMode.COMMUNICATION_MODE_ON_DEMAND

        # socket
        self._ip = ip
        self._port = port
        self._sock = None
        self._connection = None
        self._run_without_communication = run_without_communication

        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._queue = collections.deque(maxlen=queue_size)

        self.STATE_LENGTH = 42
        self.COMMAND_LENGTH = 8
        self._state = [0] * self.STATE_LENGTH
        self._command = [0] * self.COMMAND_LENGTH
        self._last_error = -10

    def _send(self, command):
        if self._run_without_communication:
            return
        data = struct.pack('!{}d'.format(self.COMMAND_LENGTH), *command)
        self._connection.send(data)

    def _recv(self):
        if self._run_without_communication:
            return [1, -10] + [0] * (self.STATE_LENGTH - 2)
        data = self._connection.recv(self.STATE_LENGTH * 8)
        state = struct.unpack('!' + 'd' * self.STATE_LENGTH, data)
        return state

    def set_timeout(self, timeout):
        assert self._connection is not None, "Uninitialized communication"
        self._connection.settimeout(timeout)

    def set_communication_mode(self, communication_mode):
        assert communication_mode in CommunicationMode, "Invalid communication type"
        self._communication_mode = communication_mode

    def set_command(self, command=None):
        # empty command
        if command is None:
            command = self._command[:]
        # validate
        assert len(command) == self.COMMAND_LENGTH, "Invalid command length"

        # on-demand
        if self._communication_mode == CommunicationMode.COMMUNICATION_MODE_ON_DEMAND:
            # send command
            self._send(command)
            # wait for the state
            state = self._recv()
            # store the state
            self._state_lock.acquire()
            self._state = state[:]
            self._state_lock.release()
            # parse error
            self._last_error = state[1]
            # return the command execution status
            return state[0] > 0

        # periodical
        elif self._communication_mode == CommunicationMode.COMMUNICATION_MODE_PERIODICAL:
            # TODO: implement periodical communication
            raise Exception("TODO")
            self._command_lock.acquire()
            self._queue.appendleft(command)
            self._command_lock.release()
        
        # unknown
        else:
            raise Exception("Unknown communication mode: {}".format(self._communication_mode))

    def get_state(self, refresh=False):
        # request state from the robot
        if refresh:
            self.set_command(None)

        self._state_lock.acquire()
        state = np.array(self._state, dtype=np.float32)
        self._state_lock.release()

        return {"joint_position": state[2:9],
                "joint_velocity": state[9:16],
                "joint_acceleration": state[16:23],
                "joint_torque": state[23:30],
                "cartesian_position": state[30:33] / 1000.0,  # mm to m
                "cartesian_orientation": state[33:36],
                "cartesian_force": state[36:39],
                "cartesian_torque": state[39:42]}

    def get_last_error(self, clear_after_read=True):
        # parse error
        error = Error(self._last_error)
        # clear error flag
        if clear_after_read:
            self._last_error = -10
        return error

    def init(self):
        if self._run_without_communication:
            print("[LibIiwaCommunication] [WARNING] Running without communication")
            return
        # create socket
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._ip, self._port))
        except:
            print(f"[LibIiwaCommunication] [ERROR] Could not create communication endpoint at {self._ip}:{self._port}")
            raise Exception(f"Could not create communication endpoint at {self._ip}:{self._port}")

        # listen for incoming connections
        self._sock.listen(1)

        # wait for a connection
        print(f"[INFO] Waiting for a connection at {self._ip}:{self._port}")
        self._connection, client_address = self._sock.accept()
        print(f"[INFO] Connection from {client_address[0]}:{client_address[1]}")

    def start(self):
        self._running = True
        while self._running:
            try:
                time.sleep(0.1)

                # send command
                self._command_lock.acquire()
                command = self._queue.pop() if len(
                    self._queue) else self._command[:]
                self._command_lock.release()
                self._send(command)

                # receive state
                state = self._recv()
                self._state_lock.acquire()
                self._state = state
                self._state_lock.release()

            except Exception as e:
                print(e)
                self._running = False
                break

        self._connection.close()
        self._sock.close()

    def stop(self):
        self._running = False

    def close(self):
        self._connection.close()
        self._sock.close()
        time.sleep(0.5)


class LibIiwa:
    def __init__(self, 
                 ip: Optional[str] = "0.0.0.0", 
                 port: Optional[int] = 12225,
                 run_without_communication: Optional[bool] = False) -> None:
        """KUKA LBR iiwa robot library

        :param ip: IP address of the library communication endpoint (default: all interfaces)
        :type ip: str, optional
        :param port: port of the library communication endpoint (default: 12225)
        :type port: int, optional
        :param run_without_communication: Run the library without creating the communication socket (default: False).
                                          Useful for testing the library without the robot
        :type run_without_communication: bool, optional
        """
        self._communication = LibIiwaCommunication(ip=ip,
                                                   port=port,
                                                   queue_size=10,
                                                   run_without_communication=run_without_communication)

    def start(self, threaded=True):
        # TODO: docstring
        self._communication.init()

        # if threaded:
        #     self._communication_thread = threading.Thread(target=self._communication.start)
        #     self._communication_thread.start()
        # else:
        #     self._communication.start()

    def shutdown(self):
        self._communication.close()

    def stop(self):
        self._communication.stop()

    def get_state(self, refresh: bool = False) -> dict:
        """Get the state of the robot

        :param refresh: If True, the state is requested from the robot otherwise the last received state is returned
        :type refresh: bool

        :return: the state of the robot
        :rtype: dict
        """
        return self._communication.get_state(refresh)

    def get_last_error(self, clear_after_read: bool = True) -> str:
        """Get the last error message from the robot

        :param clear_after_read: If True, the internal error flag will be reset after this call (default: True)
        :type clear_after_read: bool, optional

        :return: The last error message from the robot
        :rtype: str
        """
        return self._communication.get_last_error(clear_after_read)

    # motion command

    def command_joint_position(self, position: Union[List[float], np.ndarray], degrees: bool = False) -> bool:  # DONE
        """Move the robot to the specified joint position
        
        :param position: The joint position to move to
        :type position: List[float] or np.ndarray
        :param degrees: Whether the position is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the position is not a list or numpy array of length 7
        
        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # move to joint position [0, 0, 0, -1.57, 0, 1.57, 0] in radians
            >>> iiwa.command_joint_position([0, 0, 0, -1.57, 0, 1.57, 0])
            True

            >>> # move to joint position [0, 0, 0, -90, 0, 90, 0] in degrees
            >>> iiwa.command_joint_position([0, 0, 0, -90, 0, 90, 0], degrees=True)
            True

            >>> # set only the joint number 4 to -1.57 in radians without moving the other joints
            >>> iiwa.command_joint_position([np.NaN, np.NaN, np.NaN, -1.57, np.NaN, np.NaN, np.NaN])
            True
        """
        position = np.array(position, dtype=np.float32).flatten()
        assert position.size == 7, "Invalid position length"
        if degrees:
            position = np.radians(position)
        command = [COMMAND_JOINT_POSITION] + position.tolist() + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)

    def command_cartesian_pose(self, 
                               position: Union[List[float], np.ndarray], 
                               orientation: Union[List[float], np.ndarray] = [np.NaN, np.NaN, np.NaN],
                               millimeters: bool = False,
                               degrees: bool = False) -> bool:  # DONE
        """Move the robot to the specified cartesian pose

        :param position: The cartesian position to move to
        :type position: List[float] or np.ndarray
        :param orientation: The cartesian orientation to move to (default: [np.NaN, np.NaN, np.NaN])
        :type orientation: List[float] or np.ndarray
        :param millimeters: Whether the position is in millimeters or meters (default: meters)
        :type millimeters: bool
        :param degrees: Whether the orientation is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the position is not a list or numpy array of length 3
        :raises AssertionError: If the orientation is not a list or numpy array of length 3

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # move to cartesian pose [0.65, 0, 0.2] in meters withouth changing the orientation
            >>> iiwa.command_cartesian_pose([0.65, 0, 0.2])
            True

            >>> # move to cartesian pose [650, 0, 200] in millimeters withouth changing the orientation
            >>> iiwa.command_cartesian_pose([650, 0, 200], millimeters=True)
            True
        """
        # TODO: improve example
        position = np.array(position, dtype=np.float32).flatten()
        orientation = np.array(orientation, dtype=np.float32).flatten()
        assert len(position) == 3, "Invalid position length"
        assert len(orientation) == 3, "Invalid orientation length"
        if not millimeters:
            position *= 1000
        if degrees:
            orientation = np.radians(orientation)
        command = [COMMAND_CARTESIAN_POSE] + position.tolist() + orientation.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def command_circular_motion(self,  # DONE
                                auxiliary_position: Union[List[float], np.ndarray], 
                                end_position: Union[List[float], np.ndarray], 
                                millimeters: bool = False) -> bool:
        """Perform a circular motion

        Circular motion is deined by an auxiliary position and an end position. 
        The coordinates of the auxiliary position and end position are Cartesian and absolute.
        The current frame orientation will be used to calculate the circular motion

        :param auxiliary_position: The auxiliary cartesian position with respect to World
        :type auxiliary_position: List[float] or np.ndarray
        :param end_position: The end cartesian position with respect to World
        :type end_position: List[float] or np.ndarray
        :param millimeters: Whether the position is in millimeters or meters (default: meters)
        :type millimeters: bool

        :raises AssertionError: If the auxiliary position is not a list or numpy array of length 3
        :raises AssertionError: If the end position is not a list or numpy array of length 3

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # assuming the robot this joint configuration in degrees: [0, 0, 0, -90, 0, 90, 0]
            >>> # perfom a circular motion to [0.5, 0.2, 0.65] with auxiliary position [0.6, 0.1, 0.65]
            >>> iiwa.command_circular_motion([0.6, 0.1, 0.65], [0.5, 0.2, 0.65])
            True

            >>> # same circular motion in millimeters
            >>> iiwa.command_circular_motion([0.6, 0.1, 0.65], [0.5, 0.2, 0.65], millimeters=True)
            True
        """
        # TODO: improve example
        auxiliary_position = np.array(auxiliary_position, dtype=np.float32).flatten()
        end_position = np.array(end_position, dtype=np.float32).flatten()
        assert len(auxiliary_position) == 3, "Invalid auxiliary position length"
        assert len(end_position) == 3, "Invalid end position length"
        if not millimeters:
            auxiliary_position *= 1000
            end_position *= 1000
        command = [COMMAND_CIRC_MOTION] + auxiliary_position.tolist() + end_position.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    # configuration commands (limits)

    def set_desired_joint_velocity_rel(self, value: float) -> bool:  # DONE
        """Define the axis-specific relative velocity (% of maximum velocity)

        :param value: The relative velocity in % of maximum velocity [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_joint_velocity_rel(0.1)
            True
        """
        assert 0 <= value <= 1, "Invalid range [0, 1]"
        command = [COMMAND_SET_DESIRED_JOINT_VELOCITY_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_joint_acceleration_rel(self, value: float) -> bool:  # DONE
        """Define the axis-specific relative acceleration (% of maximum acceleration)

        :param value: The relative acceleration in % of maximum acceleration [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_joint_acceleration_rel(0.1)
            True
        """
        assert 0 <= value <= 1, "Invalid range [0, 1]"
        command = [COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_joint_jerk_rel(self, value: float) -> bool:  # DONE
        """Define the axis-specific relative jerk (% of maximum jerk)

        :param value: The relative jerk in % of maximum jerk [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_joint_jerk_rel(0.1)
            True
        """
        assert 0 <= value <= 1, "Invalid range [0, 1]"
        command = [COMMAND_SET_DESIRED_JOINT_JERK_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_velocity(self, value: float) -> bool:  # DONE
        """Define the absolute Cartesian velocity (m/s)

        The Cartesian velocity will be automatically converted to mm/s before sending to the robot

        :param value: The Cartesian velocity in m/s (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_cartesian_velocity(10)
            True
        """
        assert value > 0, "Invalid range (0, Inf)"
        command = [COMMAND_SET_DESIRED_CARTESIAN_VELOCITY] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_acceleration(self, value: float) -> bool:  # DONE
        """Define the absolute Cartesian acceleration (m/s^2)

        The Cartesian acceleration will be automatically converted to mm/s^2 before sending to the robot

        :param value: The Cartesian acceleration in m/s^2 (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_cartesian_acceleration(10)
            True
        """
        assert value > 0, "Invalid range (0, Inf)"
        command = [COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_jerk(self, value: float) -> bool:  # DONE
        """Define the absolute Cartesian jerk (m/s^3)

        The Cartesian jerk will be automatically converted to mm/s^3 before sending to the robot

        :param value: The Cartesian jerk in m/s^3 (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_desired_cartesian_jerk(10)
            True
        """
        assert value > 0, "Invalid range (0, Inf)"
        command = [COMMAND_SET_DESIRED_CARTESIAN_JERK] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    # configuration commands (conditions)

    def set_force_condition(self,  # DONE
                            threshold: Union[List[float], np.ndarray], 
                            tolerance: Optional[Union[List[float], np.ndarray]] = [10, 10, 10]) -> bool:
        """Define the force condition (threshold and tolerance) for each Cartesian axis

        :param threshold: Maximum magnitude of force in N [0, Inf) 
        :type threshold: 3-element list or numpy.ndarray
        :param tolerance: Maximum permissible inaccuracy in N (0, Inf) (default: [10, 10, 10])
        :type tolerance: 3-element list or numpy.ndarray, optional

        :raises AssertionError: If the length of the threshold and tolerance is not equal to the number of axes
        :raises AssertionError: If the threshold is not in the range [0, Inf)
        :raises AssertionError: If the tolerance is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # force conditon for all axes (x: 10 N, y: 20 N, z: 30 N) and default tolerance
            >>> iiwa.set_force_condition([10, 20, 30])
            True

            >>> # force conditon only for z axis (15 N) and tolerance of 1 N
            >>> iiwa.set_force_condition([np.Inf, np.Inf, 15], [np.Inf, np.Inf, 1])
            True
        """
        threshold = np.array(threshold, dtype=np.float32).flatten()
        tolerance = np.array(tolerance, dtype=np.float32).flatten()
        assert threshold.size == 3, "Invalid threshold length"
        assert tolerance.size == 3, "Invalid tolerance length"
        assert np.all(threshold >= 0), "Invalid range [0, Inf)"
        assert np.all(tolerance > 0), "Invalid range (0, Inf)"
        command = [COMMAND_SET_FORCE_CONDITION] + threshold.tolist() + tolerance.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_joint_torque_condition(self,  # DONE
                                   lower_limits : Union[List[float], np.ndarray], 
                                   upper_limits : Union[List[float], np.ndarray]) -> bool:
        """Define the joint torque condition (lower and upper limits) for each joint axis

        :param lower_limits: Lower limit of torque in Nm (-Inf, Inf)
        :type lower_limits: 7-element list or numpy.ndarray
        :param upper_limits: Upper limit of torque in Nm [-Inf, Inf)
        :type upper_limits: 7-element list or numpy.ndarray

        :raises AssertionError: If the length of the lower and upper limits is not equal to the number of axes
        :raises AssertionError: If any of the lower limits is higher than the upper limits

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # same limits for all joints (min: -2.5 Nm, max: 4.0 Nm)
            >>> lower_limits = [-2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5]
            >>> upper_limits = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
            >>> iiwa.set_joint_torque_condition(lower_limits, upper_limits)
            True

            >>> # limits only for joint 4 (min: -2.5 Nm, max: 4.0 Nm)
            >>> lower_limits = [-np.Inf, -np.Inf, -np.Inf, -2.5, -np.Inf, -np.Inf, -np.Inf]
            >>> upper_limits = [np.Inf, np.Inf, np.Inf, 4.0, np.Inf, np.Inf, np.Inf]
            >>> iiwa.set_joint_torque_condition(lower_limits, upper_limits)
            True
        """
        lower_limits = np.array(lower_limits, dtype=np.float32).flatten()
        upper_limits = np.array(upper_limits, dtype=np.float32).flatten()
        assert lower_limits.size == 7, "Invalid lower_limits length"
        assert upper_limits.size == 7, "Invalid upper_limits length"
        assert (lower_limits <= upper_limits).all(), "Invalid values (lower limits > upper limits)"
        status = True
        for i in range(7):
            command = [COMMAND_SET_JOINT_TORQUE_CONDITION] + [i + 1] + [lower_limits[i]] + [upper_limits[i]] \
                + [0] * (self._communication.COMMAND_LENGTH - 4)
            status = status and self._communication.set_command(command)
        return status

    # configuration commands (impedance control)

    def set_cartesian_stiffness(self, 
                                translational : Union[List[float], np.ndarray] = [2000.0, 2000.0, 2000.0],
                                rotational : Union[List[float], np.ndarray] = [200.0, 200.0, 200.0]) -> bool:
        """Define the stiffness (translational and rotational) for Cartesian impedance control 

        :param  translational: Translational stiffness in N/m [0.0, 5000.0] (default: [2000.0, 2000.0, 2000.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational stiffness in Nm/rad [0.0, 300.0] (default: [200.0, 200.0, 200.0])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, 5000.0]
        :raises AssertionError: If the rotational is not in the range [0.0, 300.0]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0), "Invalid range [0.0, 5000.0]"
        assert np.all(translational <= 5000.0), "Invalid range [0.0, 5000.0]"
        assert np.all(rotational >= 0), "Invalid range [0.0, 300.0]"
        assert np.all(rotational <= 300.0), "Invalid range [0.0, 300.0]"
        command = [COMMAND_SET_CARTESIAN_STIFFNESS] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_damping(self,
                              translational : Union[List[float], np.ndarray] = [0.7, 0.7, 0.7],
                              rotational : Union[List[float], np.ndarray] = [0.7, 0.7, 0.7]) -> bool:
        """Define the damping (translational and rotational) for Cartesian impedance control 

        :param  translational: Translational damping [0.1, 1.0] (default: [0.7, 0.7, 0.7])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational damping [0.1, 1.0] (default: [0.7, 0.7, 0.7])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.1, 1.0]
        :raises AssertionError: If the rotational is not in the range [0.1, 1.0]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.1), "Invalid range [0.1, 1.0]"
        assert np.all(translational <= 1.0), "Invalid range [0.1, 1.0]"
        assert np.all(rotational >= 0.1), "Invalid range [0.1, 1.0]"
        assert np.all(rotational <= 1.0), "Invalid range [0.1, 1.0]"
        command = [COMMAND_SET_CARTESIAN_DAMPING] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_additional_control_force(self,
                                               translational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                                               rotational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0]) -> bool:
        """Define the additional control force (translational and rotational) for Cartesian impedance control

        :param translational: Translational additional control force in N (default: [0.0, 0.0, 0.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational additional control force in Nm (default: [0.0, 0.0, 0.0])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes

        :return: True if successful, False otherwise
        :rtype: bool
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        command = [COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_joint_stiffness(self, stiffness: Union[List[float], np.ndarray]) -> bool:
        """Define the stiffness for joint impedance control 

        :param  stiffness: Joint stiffness in Nm/rad [0.0, Inf)
        :type stiffness: 7-element list or numpy.ndarray

        :raises AssertionError: If the length of the stiffness is not equal to the number of joints
        :raises AssertionError: If the stiffness is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool
        """
        stiffness = np.array(stiffness, dtype=np.float32).flatten()
        assert stiffness.size == 7, "Invalid stiffness length"
        assert np.all(stiffness >= 0), "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_JOINT_STIFFNESS] + stiffness.tolist() + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)

    def set_joint_damping(self, damping: Union[List[float], np.ndarray]) -> bool:
        """Define the damping for joint impedance control 

        :param  damping: Joint damping [0.0, 1.0]
        :type damping: 7-element list or numpy.ndarray

        :raises AssertionError: If the length of the damping is not equal to the number of joints
        :raises AssertionError: If the damping is not in the range [0.0, 1.0]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        damping = np.array(damping, dtype=np.float32).flatten()
        assert damping.size == 7, "Invalid damping length"
        assert np.all(damping >= 0), "Invalid range [0.0, 1.0]"
        assert np.all(damping <= 1.0), "Invalid range [0.0, 1.0]"
        command = [COMMAND_SET_JOINT_DAMPING] + damping.tolist() + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)
    
    # configuration commands (motion and control)

    def set_control_interface(self, control_interface: ControlInterface) -> bool:  # DONE
        """Set the control interface

        :param control_interface: Control interface
        :type control_interface: ControlInterface

        :raises AssertionError: If the control interface is not valid

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_control_interface(libiiwa.ControlInterface.CONTROL_INTERFACE_SERVO)
            True
        """
        assert control_interface in ControlInterface, "Invalid control interface"
        command = [COMMAND_SET_CONTROL_INTERFACE] + [control_interface.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_motion_type(self, motion_type: MotionType) -> bool:  # DONE
        """Set the motion type

        :param motion_type: Motion type
        :type motion_type: MotionType

        :raises AssertionError: If the motion type is not valid

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_motion_type(libiiwa.MotionType.MOTION_TYPE_LIN)
            True
        """
        assert motion_type in MotionType, "Invalid motion type"
        command = [COMMAND_SET_MOTION_TYPE] + [motion_type.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_control_mode(self, control_mode: ControlMode) -> bool:  # DONE
        """Set the control mode

        :param control_mode: Control mode
        :type control_mode: ControlMode

        :raises AssertionError: If the control mode is not valid

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_control_mode(libiiwa.ControlMode.CONTROL_MODE_POSITION)
            True
        """
        assert control_mode in ControlMode, "Invalid control mode"
        command = [COMMAND_SET_CONTROL_MODE] + [control_mode.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)
    
    def set_execution_type(self, execution_type: ExecutionType) -> bool:  # DONE
        """Set the execution type

        :param execution_type: Execution type
        :type execution_type: ExecutionType

        :raises AssertionError: If the execution type is not valid

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_control_mode(libiiwa.ExecutionType.EXECUTION_TYPE_ASYNCHRONOUS)
            True
        """
        assert execution_type in ExecutionType, "Invalid execution type"
        command = [COMMAND_SET_EXECUTION_TYPE] + [execution_type.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    # configuration commands (communication)

    def set_communication_mode(self, communication_mode: CommunicationMode) -> bool:
        """Set the communication mode

        :param communication_mode: Communication mode
        :type communication_mode: CommunicationMode

        :raises AssertionError: If the communication mode is not valid

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert communication_mode in CommunicationMode, "Invalid communication mode"
        command = [COMMAND_SET_COMMUNICATION_MODE] + [communication_mode.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)




if __name__ == "__main__":

    iiwa = LibIiwa()
    iiwa.start()
