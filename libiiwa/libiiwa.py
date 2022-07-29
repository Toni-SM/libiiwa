from typing import List, Union

import time
import struct
import socket
import threading
import collections
import numpy as np
from enum import Enum


__all__ = [
    'LibIiwa',
    'CommunicationMode',
    'MotionType',
    'ControlInterface',
    'ControlMode',
    'ExecutionType',
    'API_VERSION',
]


API_VERSION = "0.1.0-beta"

# empty commands
COMMAND_PASS = 0

# communication modes
class CommunicationMode(Enum):
    COMMUNICATION_MODE_ON_DEMAND = 11
    COMMUNICATION_MODE_PERIODICAL = 12

# motion types
class MotionType(Enum):
    MOTION_TYPE_PTP = 21
    MOTION_TYPE_LIN = 22
    MOTION_TYPE_LIN_REL = 23
    MOTION_TYPE_CIRC = 24

# control interfaces
class ControlInterface(Enum):
    CONTROL_INTERFACE_STANDARD = 31
    CONTROL_INTERFACE_SERVO = 32
    CONTROL_INTERFACE_FRI = 33

# control modes
class ControlMode(Enum):
    CONTROL_MODE_POSITION = 41
    CONTROL_MODE_JOINT_IMPEDANCE = 42
    CONTROL_MODE_CARTESIAN_IMPEDANCE = 43
    CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE = 44

# execution type
class ExecutionType(Enum):
	EXECUTION_TYPE_ASYNCHRONOUS = 51
	EXECUTION_TYPE_SYNCHRONOUS = 52

# control command
COMMAND_JOINT_POSITION = 101
COMMAND_CARTESIAN_POSE = 102

# configuration commands
COMMAND_SET_DESIRED_JOINT_VELOCITY_REL = 201
COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL = 202
COMMAND_SET_DESIRED_JOINT_JERK_REL = 203
COMMAND_SET_DESIRED_CARTESIAN_VELOCITY = 204
COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION = 205
COMMAND_SET_DESIRED_CARTESIAN_JERK = 206
COMMAND_SET_FORCE_CONDITION = 207
COMMAND_SET_CARTESIAN_STIFFNESS = 208
COMMAND_SET_CARTESIAN_DAMPING = 209
COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE = 210
COMMAND_SET_JOINT_STIFFNESS = 211
COMMAND_SET_JOINT_DAMPING = 212

COMMAND_SET_COMMUNICATION_MODE = 301
COMMAND_SET_CONTROL_INTERFACE = 302
COMMAND_SET_MOTION_TYPE = 303
COMMAND_SET_CONTROL_MODE = 304
COMMAND_SET_EXECUTION_TYPE = 305


class LibIiwaCommunication:
    def __init__(self,
                 ip="0.0.0.0",
                 port=12225,
                 queue_size=1):

        self._running = False
        self._communication_mode = CommunicationMode.COMMUNICATION_MODE_ON_DEMAND

        # socket
        self._ip = ip
        self._port = port
        self._sock = None
        self._connection = None

        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._queue = collections.deque(maxlen=queue_size)

        self.STATE_LENGTH = 42
        self.COMMAND_LENGTH = 8
        self._state = [0] * self.STATE_LENGTH
        self._command = [0] * self.COMMAND_LENGTH

    def _send(self, command):
        data = struct.pack('!{}d'.format(self.COMMAND_LENGTH), *command)
        self._connection.send(data)

    def _recv(self):
        data = self._connection.recv(self.STATE_LENGTH * 8)
        state = struct.unpack('!' + 'd' * self.STATE_LENGTH, data)
        return state

    def set_timeout(self, timeout):
        assert self._connection is not None  # initialized communication
        self._connection.settimeout(timeout)

    def set_communication_mode(self, communication_mode):
        assert communication_mode in CommunicationMode  # invalid communication type
        self._communication_mode = communication_mode

    def set_command(self, command=None):
        # empty command
        if command is None:
            command = self._command[:]
        # validate
        assert len(command) == self.COMMAND_LENGTH  # invalid command length

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

    def init(self):
        # create socket
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._ip, self._port))
        except:
            print("[ERROR] Connection for KUKA cannot assign requested address ")
            return

        # listen for incoming connections
        self._sock.listen(1)

        # wait for a connection
        print('[INFO] Waiting for a connection...')
        self._connection, client_address = self._sock.accept()
        print('[INFO] Connection from', client_address)

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

        self._sock.close()

    def stop(self):
        self._running = False


class LibIiwa:
    def __init__(self, ip="0.0.0.0", port=12225):
        self._communication = LibIiwaCommunication(ip=ip,
                                                   port=port,
                                                   queue_size=10)

    def start(self, threaded=True):
        self._communication.init()

        # if threaded:
        #     self._communication_thread = threading.Thread(target=self._communication.start)
        #     self._communication_thread.start()
        # else:
        #     self._communication.start()

    def stop(self):
        self._communication.stop()

    def get_state(self, refresh: bool = False) -> dict:
        """Get the state of the robot

        :param refresh: if True, the state is requested from the robot otherwise the last received state is returned
        :type refresh: bool

        :return: the state of the robot
        :rtype: dict
        """
        return self._communication.get_state(refresh)

    # motion command

    def command_joint_position(self, position: Union[List[float], np.ndarray], degrees: bool = False) -> bool:
        """Move the robot to the specified joint position
        
        :param position: The joint position to move to
        :type position: List[float] or np.ndarray
        :param degrees: Whether the position is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the position is not a list or numpy array of length 7
        
        :return: True if successful, False otherwise
        :rtype: bool
        """
        position = np.array(position, dtype=np.float32).flatten()
        assert position.size == 7  # invalid length
        if degrees:
            position = np.radians(position)
        command = [COMMAND_JOINT_POSITION] + position.tolist() + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)

    def command_cartesian_pose(self, 
                               position: Union[List[float], np.ndarray], 
                               orientation: Union[List[float], np.ndarray] = [np.Inf, np.Inf, np.Inf],
                               millimeters: bool = False,
                               degrees: bool = False) -> bool:
        """Move the robot to the specified cartesian pose

        :param position: The cartesian position to move to
        :type position: List[float] or np.ndarray
        :param orientation: The cartesian orientation to move to (default: [np.Inf, np.Inf, np.Inf])
        :type orientation: List[float] or np.ndarray
        :param millimeters: Whether the position is in millimeters or meters (default: meters)
        :type millimeters: bool
        :param degrees: Whether the orientation is in degrees or radians (default: radians)
        :type degrees: bool

        :raises AssertionError: If the position is not a list or numpy array of length 3
        :raises AssertionError: If the orientation is not a list or numpy array of length 3

        :return: True if successful, False otherwise
        :rtype: bool
        """
        position = np.array(position, dtype=np.float32).flatten()
        orientation = np.array(orientation, dtype=np.float32).flatten()
        assert len(position) == 3  # invalid length
        assert len(orientation) == 3  # invalid length
        if not millimeters:
            position *= 1000
        if degrees:
            orientation = np.radians(orientation)
        command = [COMMAND_CARTESIAN_POSE] + position.tolist() + orientation.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

    # configuration commands (limits and constants)

    def set_desired_joint_velocity_rel(self, value: float) -> bool:
        """Define the axis-specific relative velocity (% of maximum velocity)

        :param value: The relative velocity in % of maximum velocity [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert 0 <= value <= 1  # invalid range
        command = [COMMAND_SET_DESIRED_JOINT_VELOCITY_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_joint_acceleration_rel(self, value: float) -> bool:
        """Define the axis-specific relative acceleration (% of maximum acceleration)

        :param value: The relative acceleration in % of maximum acceleration [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert 0 <= value <= 1  # invalid range
        command = [COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_joint_jerk_rel(self, value: float) -> bool:
        """Define the axis-specific relative jerk (% of maximum jerk)

        :param value: The relative jerk in % of maximum jerk [0, 1]
        :type value: float

        :raises AssertionError: If the value is not in the range [0, 1]

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert 0 <= value <= 1  # invalid range
        command = [COMMAND_SET_DESIRED_JOINT_JERK_REL] + [value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_velocity(self, value: float) -> bool:
        """Define the absolute Cartesian velocity (m/s)

        The Cartesian velocity will be automatically converted to mm/s before sending to the robot

        :param value: The Cartesian velocity in m/s (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert value > 0  # invalid range
        command = [COMMAND_SET_DESIRED_CARTESIAN_VELOCITY] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_acceleration(self, value: float) -> bool:
        """Define the absolute Cartesian acceleration (m/s^2)

        The Cartesian acceleration will be automatically converted to mm/s^2 before sending to the robot

        :param value: The Cartesian acceleration in m/s^2 (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert value > 0  # invalid range
        command = [COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_desired_cartesian_jerk(self, value: float) -> bool:
        """Define the absolute Cartesian jerk (m/s^3)

        The Cartesian jerk will be automatically converted to mm/s^3 before sending to the robot

        :param value: The Cartesian jerk in m/s^3 (0, Inf)
        :type value: float

        :raises AssertionError: If the value is not in the range (0, Inf)

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert value > 0  # invalid range
        command = [COMMAND_SET_DESIRED_CARTESIAN_JERK] + [value * 1000.0] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

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
        """
        threshold = np.array(threshold, dtype=np.float32).flatten()
        tolerance = np.array(tolerance, dtype=np.float32).flatten()
        assert threshold.size == 3  # invalid length
        assert tolerance.size == 3  # invalid length
        assert np.all(threshold >= 0)  # invalid range
        assert np.all(tolerance > 0)  # invalid range
        command = [COMMAND_SET_FORCE_CONDITION] + threshold.tolist() + tolerance.tolist() \
            + [0] * (self._communication.COMMAND_LENGTH - 7)
        return self._communication.set_command(command)

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
        assert translational.size == 3  # invalid length
        assert rotational.size == 3  # invalid length
        assert np.all(translational >= 0)  # invalid range
        assert np.all(translational <= 5000.0)  # invalid range
        assert np.all(rotational >= 0)  # invalid range
        assert np.all(rotational <= 300.0)  # invalid range
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
        assert translational.size == 3  # invalid length
        assert rotational.size == 3  # invalid length
        assert np.all(translational >= 0.1)  # invalid range
        assert np.all(translational <= 1.0)  # invalid range
        assert np.all(rotational >= 0.1)  # invalid range
        assert np.all(rotational <= 1.0)  # invalid range
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
        assert translational.size == 3  # invalid length
        assert rotational.size == 3  # invalid length
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
        assert stiffness.size == 7  # invalid length
        assert np.all(stiffness >= 0)  # invalid range
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
        assert damping.size == 7  # invalid length
        assert np.all(damping >= 0)  # invalid range
        assert np.all(damping <= 1.0)  # invalid range
        command = [COMMAND_SET_JOINT_DAMPING] + damping.tolist() + [0] * (self._communication.COMMAND_LENGTH - 8)
        return self._communication.set_command(command)
    
    # configuration commands (control)

    def set_control_interface(self, control_interface: ControlInterface) -> bool:
        """Set the control interface

        :param control_interface: Control interface
        :type control_interface: ControlInterface

        :raises AssertionError: If the control interface is not valid

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert control_interface in ControlInterface  # invalid control interface
        command = [COMMAND_SET_CONTROL_INTERFACE] + [control_interface.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_motion_type(self, motion_type: MotionType) -> bool:
        """Set the motion type

        :param motion_type: Motion type
        :type motion_type: MotionType

        :raises AssertionError: If the motion type is not valid

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert motion_type in MotionType  # invalid motion type
        command = [COMMAND_SET_MOTION_TYPE] + [motion_type.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)

    def set_control_mode(self, control_mode: ControlMode) -> bool:
        """Set the control mode

        :param control_mode: Control mode
        :type control_mode: ControlMode

        :raises AssertionError: If the control mode is not valid

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert control_mode in ControlMode  # invalid control mode
        command = [COMMAND_SET_CONTROL_MODE] + [control_mode.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)
    
    def set_execution_type(self, execution_type: ExecutionType) -> bool:
        """Set the execution type

        :param execution_type: Execution type
        :type execution_type: ExecutionType

        :raises AssertionError: If the execution type is not valid

        :return: True if successful, False otherwise
        :rtype: bool
        """
        assert execution_type in ExecutionType  # invalid execution type
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
        assert communication_mode in CommunicationMode  # invalid communication mode
        command = [COMMAND_SET_COMMUNICATION_MODE] + [communication_mode.value] + [0] * (self._communication.COMMAND_LENGTH - 2)
        return self._communication.set_command(command)




if __name__ == "__main__":

    iiwa = LibIiwa()
    iiwa.start()
