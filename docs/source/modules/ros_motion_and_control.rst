ROS & ROS2: Motion and control configuration
============================================

.. raw:: html
  
    <hr>

Control mode
------------

The KUKA LBR iiwa can be operated with a number of different controllers

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Allowed values (case-insensitive)
    * - :literal:`/iiwa/set_control_mode`
      - libiiwa_msgs/SetString
      - :literal:`POSITION`, :literal:`JOINT_IMPEDANCE`, :literal:`CARTESIAN_IMPEDANCE`, :literal:`CARTESIAN_SINE_IMPEDANCE`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_control_mode-1]
                    :end-before: [end-command-line-set_control_mode-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_control_mode-1]
                    :end-before: [end-python-set_control_mode-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_control_mode-1]
                    :end-before: [end-command-line-set_control_mode-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_control_mode-1]
                    :end-before: [end-python-set_control_mode-1]

.. raw:: html
  
    <hr>

Motion type
-----------

The KUKA LBR iiwa can programmed to perform different types of motion

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Allowed values (case-insensitive)
    * - :literal:`/iiwa/set_motion_type`
      - libiiwa_msgs/SetString
      - :literal:`PTP`, :literal:`LIN`, :literal:`LIN_REL`, :literal:`CIRC`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_motion_type-1]
                    :end-before: [end-command-line-set_motion_type-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_motion_type-1]
                    :end-before: [end-python-set_motion_type-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_motion_type-1]
                    :end-before: [end-command-line-set_motion_type-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_motion_type-1]
                    :end-before: [end-python-set_motion_type-1]

.. raw:: html
  
    <hr>

Control interface
-----------------

The KUKA LBR iiwa can be operated with a number of different motion classes

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Allowed values (case-insensitive)
    * - :literal:`/iiwa/set_control_interface`
      - libiiwa_msgs/SetString
      - :literal:`STANDARD`, :literal:`SERVO`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_control_interface-1]
                    :end-before: [end-command-line-set_control_interface-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_control_interface-1]
                    :end-before: [end-python-set_control_interface-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_control_interface-1]
                    :end-before: [end-command-line-set_control_interface-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_control_interface-1]
                    :end-before: [end-python-set_control_interface-1]

.. raw:: html
  
    <hr>

Execution type
--------------

Motion commands can be executed synchronously or asynchronously

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Allowed values (case-insensitive)
    * - :literal:`/iiwa/set_execution_type`
      - libiiwa_msgs/SetString
      - :literal:`ASYNCHRONOUS`, :literal:`SYNCHRONOUS`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_execution_type-1]
                    :end-before: [end-command-line-set_execution_type-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_execution_type-1]
                    :end-before: [end-python-set_execution_type-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-command-line-set_execution_type-1]
                    :end-before: [end-command-line-set_execution_type-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_motion_and_control.txt
                    :language: bash
                    :start-after: [start-python-set_execution_type-1]
                    :end-before: [end-python-set_execution_type-1]
