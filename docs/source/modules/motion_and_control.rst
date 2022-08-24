Motion and control
==================

.. raw:: html
  
    <hr>

Control mode
------------
.. KUKA_SunriseOS_111: 17.2 Available controllers – overview

The KUKA LBR iiwa can be operated with a number of different controllers

.. list-table::
    :header-rows: 1

    * - Control mode
      - Enum (:literal:`ControlMode`)
    * - **Position controller**: Execute the specified path with the maximum possible positional accuracy and without path deviation
      - :literal:`CONTROL_MODE_POSITION`
    * - **Cartesian impedance controller**: Virtual spring damper system with configurable values for stiffness and damping. This allows the robot to react in a compliant manner to external influences
      - :literal:`CONTROL_MODE_CARTESIAN_IMPEDANCE`
    * - **Cartesian impedance controller with overlaid force oscillation**: Special form of the Cartesian impedance controller. In addition to the compliant behavior, constant force setpoints and sinusoidal force oscillations can be overlaid
      - :literal:`CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE`
    * - **Axis-specific impedance controller**: Virtual spring damper system with configurable values for stiffness and damping for each axis (joint)
      - :literal:`CONTROL_MODE_JOINT_IMPEDANCE`

.. autoclass:: libiiwa.ControlMode
    :undoc-members:
    :show-inheritance:
    :members:

.. automethod:: libiiwa.LibIiwa.set_control_mode

.. raw:: html
  
    <hr>

Motion type
-----------
.. KUKA_SunriseOS_111: 15.6 Motion programming: PTP, LIN, CIRC

The KUKA LBR iiwa can programmed to perform different types of motion

.. list-table::
    :header-rows: 1

    * - Motion type
      - Enum (:literal:`MotionType`)
    * - **Point-to-point motion (PTP)**: Executes a point-to-point motion to the end point
      - :literal:`MOTION_TYPE_PTP`
    * - **Linear motion (LIN)**: Executes a linear motion to the end point
      - :literal:`MOTION_TYPE_LIN`
    * - **Linear relative motion (LIN_REL)**: Executes a linear motion relative to the end position of the previous
      - :literal:`MOTION_TYPE_LIN_REL`
    * - **Circular motion (CIRC)**: Executes a circular motion
      - :literal:`MOTION_TYPE_CIRC`

.. autoclass:: libiiwa.MotionType
    :undoc-members:
    :show-inheritance:
    :members:

.. automethod:: libiiwa.LibIiwa.set_motion_type

.. raw:: html
  
    <hr>

Control interface
-----------------
.. KUKA_SunriseOS_111
.. KUKA_SunriseServoing_116

The KUKA LBR iiwa can be operated with a number of different motion classes

.. list-table::
    :header-rows: 1

    * - Control interface
      - Enum (:literal:`ControlInterface`)
    * - **Standard**: TODO
      - :literal:`CONTROL_INTERFACE_STANDARD`
    * - **Servo motions (Servoing)**: Non-deterministic, soft real-time motions
      - :literal:`CONTROL_INTERFACE_SERVO`
    * - **Fast Research Interface**: TODO
      - :literal:`CONTROL_INTERFACE_FRI`

.. autoclass:: libiiwa.ControlInterface
    :undoc-members:
    :show-inheritance:
    :members:

.. automethod:: libiiwa.LibIiwa.set_control_interface

.. raw:: html
  
    <hr>

Execution type
--------------
.. KUKA_SunriseOS_111: 15.6.1 Synchronous and asynchronous motion execution

Motion commands can be executed synchronously or asynchronously

.. list-table::
    :header-rows: 1

    * - Execution type
      - Enum (:literal:`ExecutionType`)
    * - **Synchronous**: Motion commands are sent in steps to the real-time controller and executed (blocking)
      - :literal:`EXECUTION_TYPE_SYNCHRONOUS`
    * - **Asynchronous**: Next program line is executed directly after the motion command is sent (non-blocking)
      - :literal:`EXECUTION_TYPE_ASYNCHRONOUS`

.. autoclass:: libiiwa.ExecutionType
    :undoc-members:
    :show-inheritance:
    :members:

.. automethod:: libiiwa.LibIiwa.set_execution_type
