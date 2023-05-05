from typing import Union, List, Mapping

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

__version__ = '0.1.1-beta'
API_VERSION = "0.2.1-beta"

# application errors
class Error(Enum):
    UNKNOW_ERROR = 0
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

# Cartesian DOF
class CartesianDOF(Enum):
    """Cartesian DOF
    """
    X = 0
    Y = 1
    Z = 2
    A = 3
    B = 4
    C = 5

# Cartesian plane
class CartesianPlane(Enum):
    """Cartesian plane
    """
    XY = 0
    XZ = 1
    YZ = 2

# control command
COMMAND_STOP = 101
COMMAND_JOINT_POSITION = 102
COMMAND_CARTESIAN_POSE = 103
COMMAND_CIRC_MOTION = 104

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
COMMAND_SET_CARTESIAN_MAX_CONTROL_FORCE = 224
COMMAND_SET_CARTESIAN_MAX_CARTESIAN_VELOCITY = 225
COMMAND_SET_CARTESIAN_MAX_PATH_DEVIATION = 226
COMMAND_SET_JOINT_STIFFNESS = 227
COMMAND_SET_JOINT_DAMPING = 228
COMMAND_SET_CARTESIAN_SINE_AMPLITUDE = 229
COMMAND_SET_CARTESIAN_SINE_FREQUENCY = 230
COMMAND_SET_CARTESIAN_SINE_PHASE = 231
COMMAND_SET_CARTESIAN_SINE_BIAS = 232
COMMAND_SET_CARTESIAN_SINE_FORCE_LIMIT = 233
COMMAND_SET_CARTESIAN_SINE_POSITION_LIMIT = 234
COMMAND_SET_CARTESIAN_SINE_TOTAL_TIME = 235
COMMAND_SET_CARTESIAN_SINE_RISE_TIME = 236
COMMAND_SET_CARTESIAN_SINE_HOLD_TIME = 237
COMMAND_SET_CARTESIAN_SINE_FALL_TIME = 238
COMMAND_SET_CARTESIAN_SINE_STAY_ACTIVE_UNTIL_PATTERN_FINISHED = 239
COMMAND_SET_CARTESIAN_SINE_CREATE_DESIRED_FORCE = 240
COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN = 241
COMMAND_SET_CARTESIAN_SINE_CREATE_LISSAJOUS_PATTERN = 242
COMMAND_SET_CARTESIAN_SINE_CREATE_SPIRAL_PATTERN = 243

# configuration commands (motion and control)
COMMAND_SET_COMMUNICATION_MODE = 301
COMMAND_SET_CONTROL_INTERFACE = 302
COMMAND_SET_MOTION_TYPE = 303
COMMAND_SET_CONTROL_MODE = 304
COMMAND_SET_EXECUTION_TYPE = 305


class LibIiwaCommunication:
    def __init__(self,
                 ip: str = "0.0.0.0",
                 port: int = 12225,
                 double_precision: bool = False,
                 run_without_communication: bool = False) -> None:
        """Library communication endpoint

        :param ip: IP address of the library communication endpoint (default: all interfaces)
        :type ip: str, optional
        :param port: Port of the library communication endpoint (default: 12225)
        :type port: int, optional
        :param double_precision: Whether double precision is used for communication (default: False)
        :type double_precision: bool, optional
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

        self.STATE_LENGTH = 38
        self.COMMAND_LENGTH = 8
        self._state = [0] * self.STATE_LENGTH
        self._command = [0] * self.COMMAND_LENGTH
        self._last_error = -10

        self._data_type = 'd' if double_precision else 'f'

    def _send(self, command):
        if self._run_without_communication:
            return
        data = struct.pack('!{}{}'.format(self.COMMAND_LENGTH, self._data_type), *command)
        self._connection.send(data)

    def _recv(self):
        if self._run_without_communication:
            return [1, -10] + [0] * (self.STATE_LENGTH - 2)
        data = self._connection.recv(self.STATE_LENGTH * 8)
        state = struct.unpack('!' + self._data_type * self.STATE_LENGTH, data)
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
            self._last_error = round(state[34])
            # return the command execution status
            return round(state[0]) > 0

        # periodical
        elif self._communication_mode == CommunicationMode.COMMUNICATION_MODE_PERIODICAL:
            # TODO: implement periodical communication
            raise NotImplementedError
        
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

        return {"joint_position": state[1:8],
                "joint_velocity": state[8:15],
                "joint_torque": state[15:22],
                "cartesian_position": state[22:25] / 1000.0,  # mm to m
                "cartesian_orientation": state[25:28],
                "cartesian_force": state[28:31],
                "cartesian_torque": state[31:34],
                "last_error": Error(int(round(state[34]))),
                "has_fired_condition": bool(round(state[35])),
                "is_ready_to_move": bool(round(state[36])),
                "has_active_motion": bool(round(state[37]))}

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

    def stop(self):
        self._running = False

    def close(self):
        self._running = False
        self._connection.close()
        self._sock.close()
        time.sleep(0.5)


class LibIiwa:
    def __init__(self, 
                 ip: str = "0.0.0.0", 
                 port: int = 12225,
                 double_precision: bool = False,
                 run_without_communication: bool = False) -> None:
        """KUKA LBR iiwa robot library

        :param ip: IP address of the library communication endpoint (default: all interfaces)
        :type ip: str, optional
        :param port: Port of the library communication endpoint (default: 12225)
        :type port: int, optional
        :param run_without_communication: Run the library without creating the communication socket (default: False).
                                          Useful for testing the library without the robot
        :type run_without_communication: bool, optional

        Example::

            >>> import libiiwa
            >>>
            >>> # create instance with default parameters
            >>> iiwa = libiiwa.LibIiwa()
        """
        self._communication = LibIiwaCommunication(ip=ip,
                                                   port=port,
                                                   double_precision=double_precision,
                                                   run_without_communication=run_without_communication)
        self._communication.init()

    def shutdown(self):
        # TODO: check it
        self._communication.close()

    def stop(self):
        # TODO: check it
        self._communication.stop()

    def get_state(self, refresh: bool = False) -> Mapping[str, np.ndarray]:
        """Get the state of the robot

        :param refresh: If True, the state is requested from the robot otherwise the last received state is returned (default: False)
        :type refresh: bool, optional

        :return: The state of the robot
        :rtype: dict

        Example::

            >>> # get an updated robot state
            >>> iiwa.get_state(refresh=True)
            {'joint_position': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 'joint_velocity': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 
            'joint_acceleration': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 'joint_torque': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 
            'cartesian_position': array([0., 0., 0.], dtype=float32), 'cartesian_orientation': array([0., 0., 0.], dtype=float32), 
            'cartesian_force': array([0., 0., 0.], dtype=float32), 'cartesian_torque': array([0., 0., 0.], dtype=float32)}

            >>> # get the last updated robot state
            >>> iiwa.get_state(refresh=False)  # or just `iiwa.get_state()`
            {'joint_position': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 'joint_velocity': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 
            'joint_acceleration': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 'joint_torque': array([0., 0., 0., 0., 0., 0., 0.], dtype=float32), 
            'cartesian_position': array([0., 0., 0.], dtype=float32), 'cartesian_orientation': array([0., 0., 0.], dtype=float32), 
            'cartesian_force': array([0., 0., 0.], dtype=float32), 'cartesian_torque': array([0., 0., 0.], dtype=float32)}
        """
        return self._communication.get_state(refresh)

    def get_last_error(self, clear_after_read: bool = True) -> Error:
        """Get the last error message from the robot

        If there is no error, ``Error.NO_ERROR`` enum will be returned

        :param clear_after_read: If True, the internal error flag will be reset after this call (default: True).
        :type clear_after_read: bool, optional

        :return: The last error message from the robot
        :rtype: Error

        Example::

            >>> # no errors
            >>> iiwa.get_last_error()
            <Error.NO_ERROR: -10>
        """
        return self._communication.get_last_error(clear_after_read)

    # motion commands

    def command_stop(self) -> bool:
        """Stop the robot

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # stop the robot
            >>> iiwa.command_stop()
            True
        """
        command = [COMMAND_STOP] + [0] * (self._communication.COMMAND_LENGTH - 1)
        return self._communication.set_command(command)

    def command_joint_position(self, position: Union[List[float], np.ndarray], degrees: bool = False) -> bool:  # DONE
        """Move the robot to the specified joint position
        
        :param position: The joint position to move to
        :type position: 7-element list or np.ndarray
        :param degrees: Whether the position is in degrees or radians (default: radians)
        :type degrees: bool, optional

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
        :type position: 3-element list or np.ndarray
        :param orientation: The cartesian orientation to move to (default: [np.NaN, np.NaN, np.NaN])
        :type orientation: 3-element list or np.ndarray, optional
        :param millimeters: Whether the position is in millimeters or meters (default: meters)
        :type millimeters: bool, optional
        :param degrees: Whether the orientation is in degrees or radians (default: radians)
        :type degrees: bool, optional

        :raises AssertionError: If the position is not a list or numpy array of length 3
        :raises AssertionError: If the orientation is not a list or numpy array of length 3

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> # move to cartesian pose [0.65, 0, 0.2] in meters without changing the orientation
            >>> iiwa.command_cartesian_pose([0.65, 0, 0.2])
            True

            >>> # move to cartesian pose [650, 0, 200] in millimeters without changing the orientation
            >>> iiwa.command_cartesian_pose([650, 0, 200], millimeters=True)
            True
        """
        # TODO: improve example with NaN
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

        Circular motion is defined by an auxiliary position and an end position.
        The coordinates of the auxiliary position and end position are Cartesian and absolute.
        The current frame orientation will be used to calculate the circular motion

        :param auxiliary_position: The auxiliary cartesian position with respect to World
        :type auxiliary_position: 3-element list or np.ndarray
        :param end_position: The end cartesian position with respect to World
        :type end_position: 3-element list or np.ndarray
        :param millimeters: Whether the position is in millimeters or meters (default: meters)
        :type millimeters: bool, optional

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

    def reset_conditions(self) -> bool:
        """Reset all conditions

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.reset_conditions()
            True
        """
        status = self.set_force_condition(threshold=[np.NaN, np.NaN, np.NaN])
        status = status and self.set_joint_torque_condition(lower_limits=[np.NaN, np.NaN, np.NaN, np.NaN, np.NaN, np.NaN, np.NaN],
                                                            upper_limits=[np.NaN, np.NaN, np.NaN, np.NaN, np.NaN, np.NaN, np.NaN])
        return status

    def set_force_condition(self,
                            threshold: Union[List[float], np.ndarray], 
                            tolerance: Union[List[float], np.ndarray] = [10, 10, 10]) -> bool:
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

            >>> # force condition for all axes (x: 10 N, y: 20 N, z: 30 N) and default tolerance
            >>> iiwa.set_force_condition([10, 20, 30])
            True

            >>> # force condition only for z axis (15 N) and tolerance of 1 N
            >>> iiwa.set_force_condition([np.NaN, np.NaN, 15], [np.NaN, np.NaN, 1])
            True
        """
        threshold = np.array(threshold, dtype=np.float32).flatten()
        tolerance = np.array(tolerance, dtype=np.float32).flatten()
        assert threshold.size == 3, "Invalid threshold length"
        assert tolerance.size == 3, "Invalid tolerance length"
        assert np.all(threshold[np.logical_not(np.isnan(threshold))] >= 0), "Invalid range [0, Inf)"
        assert np.all(tolerance[np.logical_not(np.isnan(tolerance))] >= 0), "Invalid range (0, Inf)"
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
            >>> lower_limits = [np.NaN, np.NaN, np.NaN, -2.5, np.NaN, np.NaN, np.NaN]
            >>> upper_limits = [np.NaN, np.NaN, np.NaN, 4.0, np.NaN, np.NaN, np.NaN]
            >>> iiwa.set_joint_torque_condition(lower_limits, upper_limits)
            True
        """
        lower_limits = np.array(lower_limits, dtype=np.float32).flatten()
        upper_limits = np.array(upper_limits, dtype=np.float32).flatten()
        assert lower_limits.size == 7, "Invalid lower_limits length"
        assert upper_limits.size == 7, "Invalid upper_limits length"
        indexes = np.logical_not(np.logical_or(np.isnan(lower_limits), np.isnan(upper_limits)))
        assert (lower_limits[indexes] <= upper_limits[indexes]).all(), "Invalid values (lower limits > upper limits)"
        status = True
        for i in range(7):
            command = [COMMAND_SET_JOINT_TORQUE_CONDITION] + [i] + [lower_limits[i]] + [upper_limits[i]] \
                + [0] * (self._communication.COMMAND_LENGTH - 4)
            status = status and self._communication.set_command(command)
        return status

    # configuration commands (impedance control)

    def set_cartesian_stiffness(self, 
                                translational : Union[List[float], np.ndarray] = [2000.0, 2000.0, 2000.0],
                                rotational : Union[List[float], np.ndarray] = [200.0, 200.0, 200.0],
                                null_space: float = 100) -> bool:
        """Define the stiffness (translational and rotational) for Cartesian impedance control 

        .. note::
        
            This method also affects the Cartesian impedance controller with overlaid force oscillation

        :param  translational: Translational stiffness in N/m [0.0, 5000.0] (default: [2000.0, 2000.0, 2000.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational stiffness in Nm/rad [0.0, 300.0] (default: [200.0, 200.0, 200.0])
        :type rotational: 3-element list or numpy.ndarray
        :type null_space: Spring stiffness of the redundancy degree of freedom in Nm/rad [0.0, Inf) (default: 100)
        :type null_space: float

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, 5000.0]
        :raises AssertionError: If the rotational is not in the range [0.0, 300.0]
        :raises AssertionError: If the null_space is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> translational = [2000.0, 3000.0, 4000.0]
            >>> rotational = [50.0, 100.0, 150.0]
            >>> iiwa.set_cartesian_stiffness(translational, rotational, null_space=100)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0), "Invalid range [0.0, 5000.0]"
        assert np.all(translational <= 5000.0), "Invalid range [0.0, 5000.0]"
        assert np.all(rotational >= 0), "Invalid range [0.0, 300.0]"
        assert np.all(rotational <= 300.0), "Invalid range [0.0, 300.0]"
        assert null_space >= 0, "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_STIFFNESS] + translational.tolist() + rotational.tolist() \
            + [null_space] + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)

    def set_cartesian_damping(self,
                              translational : Union[List[float], np.ndarray] = [0.7, 0.7, 0.7],
                              rotational : Union[List[float], np.ndarray] = [0.7, 0.7, 0.7],
                              null_space: float = 0.7) -> bool:
        """Define the damping (translational and rotational) for Cartesian impedance control 

        .. note::
        
            This method also affects the Cartesian impedance controller with overlaid force oscillation

        :param  translational: Translational damping [0.1, 1.0] (default: [0.7, 0.7, 0.7])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational damping [0.1, 1.0] (default: [0.7, 0.7, 0.7])
        :type rotational: 3-element list or numpy.ndarray
        :type null_space: Spring damping of the redundancy degree of freedom [0.3, 1.0] (default: 0.7)
        :type null_space: float

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.1, 1.0]
        :raises AssertionError: If the rotational is not in the range [0.1, 1.0]
        :raises AssertionError: If the null_space is not in the range [0.3, 1.0]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> translational = [0.2, 0.3, 0.4]
            >>> rotational = [0.5, 0.6, 0.7]
            >>> iiwa.set_cartesian_damping(translational, rotational, null_space=0.5)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.1), "Invalid range [0.1, 1.0]"
        assert np.all(translational <= 1.0), "Invalid range [0.1, 1.0]"
        assert np.all(rotational >= 0.1), "Invalid range [0.1, 1.0]"
        assert np.all(rotational <= 1.0), "Invalid range [0.1, 1.0]"
        assert null_space >= 0.3 and null_space <= 1.0, "Invalid range [0.3, 1.0]"
        command = [COMMAND_SET_CARTESIAN_DAMPING] + translational.tolist() + rotational.tolist() \
            + [null_space] + [0] * (self._communication.COMMAND_LENGTH - 8)
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

        Example::

            >>> translational = [-10.0, -20.0, -30.0]
            >>> rotational = [10.0, 20.0, 30.0]
            >>> iiwa.set_cartesian_additional_control_force(translational, rotational)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        command = [COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_max_control_force(self,
                                        translational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                        rotational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                        add_stop_condition: bool = False) -> bool:
        """Define the limitation of the maximum force (translational) / torque (rotational) on the TCP

        .. note::
        
            This method also affects the Cartesian impedance controller with overlaid force oscillation

        :param translational: Maximum force in N (default: [1e6, 1e6, 1e6])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Maximum torque in Nm (default: [1e6, 1e6, 1e6])
        :type rotational: 3-element list or numpy.ndarray
        :param add_stop_condition: Whether to cancel the motion if the maximum force at the TCP is exceeded (default: False)
        :type add_stop_condition: bool

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # set Cartesian maximum control force and cancel the motion if the maximum force is exceeded
            >>> translational = [10.0, 20.0, 30.0]
            >>> rotational = [10.0, 20.0, 30.0]
            >>> iiwa.set_cartesian_max_control_force(translational, rotational, add_stop_condition=True)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_MAX_CONTROL_FORCE] + translational.tolist() + rotational.tolist() \
            + [float(add_stop_condition)] + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)

    def set_cartesian_max_velocity(self,
                                   translational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                   rotational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6]) -> bool:
        """Define the maximum Cartesian velocity at which motion is aborted if the limit is exceeded

        .. note::
        
            This method also affects the Cartesian impedance controller with overlaid force oscillation

        The Cartesian velocity will be automatically converted to mm/s before sending to the robot.
        Assign high values to the degrees of freedom that are not to be limited

        :param translational: Maximum Cartesian velocity in m/s (default: [1e6, 1e6, 1e6])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Maximum Cartesian velocity in rad/s (default: [1e6, 1e6, 1e6])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # abort the motion for a maximum Cartesian velocity of 10  m/s in the z-axis
            >>> translational = [1e6, 1e6, 10.0]
            >>> iiwa.set_cartesian_max_velocity(translational)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten() * 1000.0
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_MAX_CARTESIAN_VELOCITY] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_max_path_deviation(self,
                                         translational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                         rotational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],          
                                         millimeters: bool = False,
                                         degrees: bool = False) -> bool:
        """Define the maximum permissible Cartesian path deviation at which motion is aborted if the limit is exceeded

        .. note::
        
            This method also affects the Cartesian impedance controller with overlaid force oscillation

        Assign high values to the degrees of freedom that are not to be limited

        :param translational: Maximum path deviation (default: [1e6, 1e6, 1e6])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Maximum path deviation (default: [1e6, 1e6, 1e6])
        :type rotational: 3-element list or numpy.ndarray
        :param millimeters: Whether the translation is in millimeters or meters (default: meters)
        :type millimeters: bool
        :param degrees: Whether the rotation is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # abort the motion for a maximum Cartesian path deviation of 0.1 m in the y-axis
            >>> translational = [1e6, 0.1, 1e6]
            >>> iiwa.set_cartesian_max_velocity(translational)
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        if not millimeters:
            translational *= 1000
        if degrees:
            rotational = np.radians(rotational)
        command = [COMMAND_SET_CARTESIAN_MAX_PATH_DEVIATION] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_amplitude(self,
                                translational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                                rotational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0]) -> bool:
        """Define the amplitude of the force oscillation

        :param translational: Translational amplitude in N (default: [0.0, 0.0, 0.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational amplitude in Nm (default: [0.0, 0.0, 0.0])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_amplitude()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_AMPLITUDE] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)
    
    def set_cartesian_frequency(self,
                                translational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                                rotational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0]) -> bool:
        """Define the frequency of the force oscillation

        :param translational: Translational frequency in Hz (default: [0.0, 0.0, 0.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational frequency in Hz (default: [0.0, 0.0, 0.0])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, 15.0]
        :raises AssertionError: If the rotational is not in the range [0.0, 15.0]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_frequency()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, 15.0]"
        assert np.all(translational <= 15.0), "Invalid range [0.0, 15.0]"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, 15.0]"
        assert np.all(rotational <= 15.0), "Invalid range [0.0, 15.0]"
        command = [COMMAND_SET_CARTESIAN_SINE_FREQUENCY] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_phase(self,
                            translational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                            rotational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                            degrees: bool = False) -> bool:
        """Define the phase offset of the force oscillation at the start of the force overlay

        :param translational: Translational phase (default: [0.0, 0.0, 0.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational phase (default: [0.0, 0.0, 0.0])
        :type rotational: 3-element list or numpy.ndarray
        :param degrees: Whether the rotational is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_phase()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        if not degrees:  # expected unit: degrees
            rotational = np.degrees(rotational)  
        command = [COMMAND_SET_CARTESIAN_SINE_PHASE] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_bias(self,
                           translational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0],
                           rotational : Union[List[float], np.ndarray] = [0.0, 0.0, 0.0]) -> bool:
        """Define a constant force overlaid (bias) in addition to the overlaid force oscillation

        :param translational: Translational bias in N (default: [0.0, 0.0, 0.0])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational bias in Nm (default: [0.0, 0.0, 0.0])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_bias()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        command = [COMMAND_SET_CARTESIAN_SINE_BIAS] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_force_limit(self,
                                  translational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                  rotational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6]) -> bool:
        """Define the force limit of the force oscillation

        :param translational: Translational force limit in N (default: [1e6, 1e6, 1e6])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational force limit in Nm (default: [1e6, 1e6, 1e6])
        :type rotational: 3-element list or numpy.ndarray

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_force_limit()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_FORCE_LIMIT] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_position_limit(self,
                                     translational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6],
                                     rotational : Union[List[float], np.ndarray] = [1e6, 1e6, 1e6], 
                                     millimeters: bool = False,
                                     degrees: bool = False) -> bool:
        """Define the maximum deflection due to the force oscillation

        :param translational: Translational maximum deflection (default: [1e6, 1e6, 1e6])
        :type translational: 3-element list or numpy.ndarray
        :param rotational: Rotational maximum deflection (default: [1e6, 1e6, 1e6])
        :type rotational: 3-element list or numpy.ndarray
        :param millimeters: Whether the translation is in millimeters or meters (default: meters)
        :type millimeters: bool
        :param degrees: Whether the rotation is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the length of the translational and rotational is not equal to the number of axes
        :raises AssertionError: If the translational is not in the range [0.0, Inf)
        :raises AssertionError: If the rotational is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # TODO
            >>> 
            >>> iiwa.set_cartesian_position_limit()
            True
        """
        translational = np.array(translational, dtype=np.float32).flatten()
        rotational = np.array(rotational, dtype=np.float32).flatten()
        assert translational.size == 3, "Invalid translational length"
        assert rotational.size == 3, "Invalid rotational length"
        assert np.all(translational >= 0.0), "Invalid range [0.0, Inf)"
        assert np.all(rotational >= 0.0), "Invalid range [0.0, Inf)"
        if not millimeters:
            translational *= 1000
        if degrees:
            rotational = np.radians(rotational)
        command = [COMMAND_SET_CARTESIAN_SINE_POSITION_LIMIT] + translational.tolist() + rotational.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    def set_cartesian_total_time(self, value: float) -> bool:
        """Define the overall duration of the force oscillation

        :param value: Duration of the force oscillation in seconds
        :type value: float

        :raises AssertionError: If the value is not in the range [0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_cartesian_total_time(60)
            True
        """
        assert value >= 0, "Invalid range [0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_TOTAL_TIME] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_cartesian_rise_time(self, value: float) -> bool:
        """Define the rise time of the force oscillation

        :param value: Rise time of the force oscillation in seconds
        :type value: float

        :raises AssertionError: If the value is not in the range [0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_cartesian_rise_time(5)
            True
        """
        assert value >= 0, "Invalid range [0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_RISE_TIME] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_cartesian_hold_time(self, value: float) -> bool:
        """Define the hold time of the force oscillation

        :param value: Hold time of the force oscillation in seconds
        :type value: float

        :raises AssertionError: If the value is not in the range [0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_cartesian_hold_time(50)
            True
        """
        assert value >= 0, "Invalid range [0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_HOLD_TIME] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_cartesian_fall_time(self, value: float) -> bool:
        """Define the fall time of the force oscillation

        :param value: Fall time of the force oscillation in seconds
        :type value: float

        :raises AssertionError: If the value is not in the range [0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_cartesian_fall_time(5)
            True
        """
        assert value >= 0, "Invalid range [0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_FALL_TIME] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_cartesian_stay_active_until_pattern_finished(self, active: bool) -> bool:
        """Define whether the oscillation is terminated or continued after the end of the motion

        :param active: Whether the oscillation is continued after the end of the motion
        :type active: bool

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> iiwa.set_cartesian_stay_active_until_pattern_finished(True)
            True
        """
        command = [COMMAND_SET_CARTESIAN_SINE_STAY_ACTIVE_UNTIL_PATTERN_FINISHED] + [float(active)] \
            + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def overlay_desired_force(self,
                              dof: CartesianDOF,
                              force: float,          
                              stiffness: float) -> bool:
        """Overlay a constant force, in one Cartesian direction, that does not change over time

        .. note::

            Because this method creates a new controller instance, any previous impedance control configuration calls will have no effect. 
            If applicable, configure the impedance controller after calling this method

        :param dof: Cartesian DOF
        :type dof: CartesianDOF
        :param force: Overlaid constant force.
                      In N for translational DOFs, in Nm for rotational DOFs
        :type force: float
        :param stiffness: Stiffness value for the specified DOF.
                          In N/m for translational DOFs, in Nm/rad for rotational DOFs
        :type stiffness: float

        :raises AssertionError: If the force is not in the range [0.0, Inf)
        :raises AssertionError: If the stiffness is not in the range [0.0, 5000] for translational DOFs or [0.0, 300] for rotational DOFs

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # exert an additional pressing constant force of 5 N in the Z direction
            >>> iiwa.overlay_desired_force(CartesianDOF.Z, 5.0, 4000.0)
            True
        """
        assert force >= 0.0, "Invalid range [0.0, Inf)"
        if dof.value <= 2:
            assert stiffness >= 0.0 and stiffness <= 5000.0, "Invalid range [0.0, 5000.0]"
        else:
            assert stiffness >= 0.0 and stiffness <= 300.0, "Invalid range [0.0, 300.0]"
        command = [COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN] + [dof.value, force, stiffness] \
            + [0] * (self._communication.COMMAND_LENGTH - 4)
        return self._communication.set_command(command)

    def overlay_sine_pattern(self,
                             dof: CartesianDOF,
                             frequency: float,          
                             amplitude: float,
                             stiffness: float) -> bool:
        """Overlay a simple force oscillation in one Cartesian direction

        .. note::

            Because this method creates a new controller instance, any previous impedance control configuration calls will have no effect. 
            If applicable, configure the impedance controller after calling this method

        :param dof: Cartesian DOF
        :type dof: CartesianDOF
        :param frequency: Frequency of the oscillation in Hz
        :type frequency: float
        :param amplitude: Amplitude of the oscillation which is overlaid in the direction of the specified DOF.
                          In N for translational DOFs, in Nm for rotational DOFs
        :type amplitude: float
        :param stiffness: Stiffness value for the specified DOF.
                          In N/m for translational DOFs, in Nm/rad for rotational DOFs
        :type stiffness: float

        :raises AssertionError: If the frequency is not in the range [0.0, 15.0]
        :raises AssertionError: If the amplitude is not in the range [0.0, Inf)
        :raises AssertionError: If the stiffness is not in the range [0.0, 5000] for translational DOFs or [0.0, 300] for rotational DOFs

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # move in a wave path with a deflection of about 0.10 meters (derived from amplitude and stiffness) 
            # and a frequency of 2 Hz in the X direction
            >>> iiwa.overlay_sine_pattern(CartesianDOF.X, 2.0, 50.0, 500.0)
            True
        """
        assert frequency >= 0.0 and frequency <= 15.0, "Invalid range [0.0, 15.0]"
        assert amplitude >= 0.0, "Invalid range [0.0, Inf)"
        if dof.value <= 2:
            assert stiffness >= 0.0 and stiffness <= 5000.0, "Invalid range [0.0, 5000.0]"
        else:
            assert stiffness >= 0.0 and stiffness <= 300.0, "Invalid range [0.0, 300.0]"
        command = [COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN] + [dof.value, frequency, amplitude, stiffness] \
            + [0] * (self._communication.COMMAND_LENGTH - 5)
        return self._communication.set_command(command)

    def overlay_lissajous_pattern(self,
                                  plane: CartesianPlane,
                                  frequency: float,          
                                  amplitude: float,
                                  stiffness: float) -> bool:
        """Overlay a 2-dimensional oscillation in one plane

        .. note::

            Because this method creates a new controller instance, any previous impedance control configuration calls will have no effect. 
            If applicable, configure the impedance controller after calling this method

        The parameters of the second DOF of the plane are calculated to generate a Lissajous curve

        - amplitude ratio (1st DOF : 2nd DOF): 1 : 1
        - frequency ratio (1st DOF : 2nd DOF): 1 : 0.4
        - phase offset between 1st and 2nd DOF: pi / 2

        :param dof: Cartesian plane
        :type dof: CartesianPlane
        :param frequency: Frequency of the oscillation for the first DOF of the plane in Hz
        :type frequency: float
        :param amplitude: Amplitude of the oscillation for both DOFs in N
        :type amplitude: float
        :param stiffness: Stiffness value for both DOFs in N/m
        :type stiffness: float

        :raises AssertionError: If the frequency is not in the range [0.0, 15.0]
        :raises AssertionError: If the amplitude is not in the range [0.0, Inf)
        :raises AssertionError: If the stiffness is not in the range [0.0, 5000]

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # generate an oscillation with a frequency ratio X : Y of 1 : 0.4
            >>> iiwa.overlay_lissajous_pattern(CartesianPlane.XY, 10.0, 50.0, 500.0)
            True
        """
        assert frequency >= 0.0 and frequency <= 15.0, "Invalid range [0.0, 15.0]"
        assert amplitude >= 0.0, "Invalid range [0.0, Inf)"
        assert stiffness >= 0.0 and stiffness <= 5000.0, "Invalid range [0.0, 5000.0]"
        command = [COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN] + [plane.value, frequency, amplitude, stiffness] \
            + [0] * (self._communication.COMMAND_LENGTH - 5)
        return self._communication.set_command(command)

    def overlay_spiral_pattern(self,
                               plane: CartesianPlane,
                               frequency: float,          
                               amplitude: float,
                               stiffness: float,
                               total_time: float) -> bool:
        """Overlay a spiral-shaped force oscillation in one plane

        .. note::

            Because this method creates a new controller instance, any previous impedance control configuration calls will have no effect. 
            If applicable, configure the impedance controller after calling this method

        The force characteristic is created by overlaying 2 sinusoidal force oscillations

        - phase offset between 1st and 2nd DOF: pi / 2

        :param dof: Cartesian plane
        :type dof: CartesianPlane
        :param frequency: Frequency of the oscillation for both DOFs in Hz
        :type frequency: float
        :param amplitude: Amplitude of the oscillation for both DOFs in N
        :type amplitude: float
        :param stiffness: Stiffness value for both DOFs in N/m
        :type stiffness: float
        :param total_time: Total time for the spiral-shaped oscillation in seconds
        :type total_time: float

        :raises AssertionError: If the frequency is not in the range [0.0, 15.0]
        :raises AssertionError: If the amplitude is not in the range [0.0, Inf)
        :raises AssertionError: If the stiffness is not in the range [0.0, 5000]
        :raises AssertionError: If the total_time is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            # increase the force in a spiral to a maximum of 100 N.
            # The force curve should rotate once per second around the starting point of the spiral.
            # The force spiral should rise and fall within 10 seconds
            >>> iiwa.overlay_spiral_pattern(CartesianPlane.XY, 1.0, 100.0, 500.0, 10)
            True
        """
        assert frequency >= 0.0 and frequency <= 15.0, "Invalid range [0.0, 15.0]"
        assert amplitude >= 0.0, "Invalid range [0.0, Inf)"
        assert stiffness >= 0.0 and stiffness <= 5000.0, "Invalid range [0.0, 5000.0]"
        assert total_time >= 0.0, "Invalid range [0.0, Inf)"
        command = [COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN] + [plane.value, frequency, amplitude, stiffness, total_time] \
            + [0] * (self._communication.COMMAND_LENGTH - 6)
        return self._communication.set_command(command)

    def set_joint_stiffness(self, stiffness: Union[List[float], np.ndarray]) -> bool:
        """Define the stiffness for joint impedance control 

        :param  stiffness: Joint stiffness in Nm/rad [0.0, Inf)
        :type stiffness: 7-element list or numpy.ndarray

        :raises AssertionError: If the length of the stiffness is not equal to the number of joints
        :raises AssertionError: If the stiffness is not in the range [0.0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool

        Example::

            >>> stiffness = [400.0, 350.0, 300.0, 250.0, 200.0, 150.0, 100.0]
            >>> iiwa.set_joint_stiffness(stiffness)
            True
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

        Example::

            >>> damping = [0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
            >>> iiwa.set_joint_damping(damping)
            True
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

    iiwa = LibIiwa(run_without_communication=True)
