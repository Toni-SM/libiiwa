Robot state and errors
======================

.. raw:: html
  
    <hr>

Robot state
-----------

.. list-table::
    :header-rows: 1

    * - State
      - Dictionary key
      - Unit
      - Data type
    * - Axis-specific (joint) actual position
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
    * - External Cartesian forces
      - :literal:`cartesian_force`
      - :literal:`N`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`
    * - External Cartesian torques
      - :literal:`cartesian_torque`
      - :literal:`Nm`
      - :literal:`np.ndarray` :literal:`shape=(3,)` :literal:`dtype=np.float32`

.. automethod:: libiiwa.LibIiwa.get_state

.. raw:: html
  
    <hr>

Errors
------

.. list-table::
    :header-rows: 1

    * - Operation errors
      - Enum (:literal:`Error`)
    * - TODO
      - :literal:`INVALID_CONFIGURATION_ERROR`
    * - TODO
      - :literal:`VALIDATION_FOR_IMPEDANCE_ERROR`
    * - TODO
      - :literal:`ASYNCHRONOUS_MOTION_ERROR`
    * - TODO
      - :literal:`SYNCHRONOUS_MOTION_ERROR`
    * - TODO
      - :literal:`INVALID_JOINT_ERROR`
    * - TODO
      - :literal:`VALUE_ERROR`

.. autoclass:: libiiwa.Error
    :undoc-members:
    :show-inheritance:
    :members:

.. automethod:: libiiwa.LibIiwa.get_last_error
