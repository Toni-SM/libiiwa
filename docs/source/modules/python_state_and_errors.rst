State and errors
================

.. |_| unicode:: 0xA0 
    :trim:

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>
    <style>.section > h3 { display: none; }</style>

Robot state
-----------

.. list-table::
    :header-rows: 1

    * - State
      - Dictionary key
      - Unit
      - Data type
    * - Axis\ |_| \-\ |_| \specific\ |_| \(joint)\ |_| \actual\ |_| \position
      - :literal:`joint_position`
      - :literal:`radians`
      - :literal:`np.ndarray` :literal:`shape=(7,)` :literal:`dtype=np.float32`
    * - Axis-specific (joint) actual velocity internally calculated as the position difference in 1/100 second interval
      - :literal:`joint_velocity`
      - :literal:`radians/s`
      - :literal:`np.ndarray` :literal:`shape=(7,)` :literal:`dtype=np.float32`
    * - External acting torques without the component resulting from the robot weight and mass inertias during motion
      - :literal:`joint_torque`
      - :literal:`Nm`
      - :literal:`np.ndarray` :literal:`shape=(7,)` :literal:`dtype=np.float32`
    * - Cartesian position (X,Y,Z)
      - :literal:`cartesian_position`
      - :literal:`m`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`
    * - Cartesian orientation (A,B,C)
      - :literal:`cartesian_orientation`
      - :literal:`radians`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`
    * - External Cartesian forces (X,Y,Z)
      - :literal:`cartesian_force`
      - :literal:`N`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`
    * - External Cartesian torques (A,B,C)
      - :literal:`cartesian_torque`
      - :literal:`Nm`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`

    * - Last registered error code. See `Errors <#id1>`_
      - :literal:`last_error`
      - 
      - :literal:`libiiwa.Error`
    * - Whether motion has terminated due to a break condition
      - :literal:`has_fired_condition`
      - 
      - :literal:`bool`
    * - Whether the robot is ready for motion. A true value does not necessarily mean that the brakes are open and that the robot is under servo control
      - :literal:`is_ready_to_move`
      - 
      - :literal:`bool`
    * - Whether the robot is active. It does not provide any information on whether the robot is currently in motion (a false value does not necessarily mean that the robot is stationary)
      - :literal:`has_active_motion`
      - 
      - :literal:`bool`

get_state
^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.get_state

.. raw:: html
  
    <hr>

Errors
------

.. list-table::
    :header-rows: 1

    * - Operation errors
      - Enum (:literal:`Error`)
    * - The robot has been configured to work with incompatible control settings
      - :literal:`INVALID_CONFIGURATION_ERROR`
    * - Validation of the load model for impedance control to use servo motions has failed
      - :literal:`VALIDATION_FOR_IMPEDANCE_ERROR`
    * - Asynchronous motion command execution failed
      - :literal:`ASYNCHRONOUS_MOTION_ERROR`
    * - Synchronous motion command execution failed
      - :literal:`SYNCHRONOUS_MOTION_ERROR`
    * - Joint subscript is out of range
      - :literal:`INVALID_JOINT_ERROR`
    * - An operation or function receives an argument that has the right type but an inappropriate value
      - :literal:`VALUE_ERROR`
    * - No error. This is not a guarantee that there are no errors
      - :literal:`NO_ERROR`

.. autoclass:: libiiwa.Error
    :undoc-members:
    :show-inheritance:
    :members:

get_last_error
^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.get_last_error
