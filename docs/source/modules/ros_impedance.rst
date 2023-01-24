ROS & ROS2: Impedance control
=============================

.. |_| unicode:: 0xA0 
    :trim:

Impedance control allows implementation of compliant robot behavior

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Joint impedance control
-----------------------

Set joint stiffness
^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_joint_stiffness`
      - `libiiwa_msgs/SetArray <ros.html#setarray-srv>`_
      - [0.0, Inf)
      - :literal:`Nm/rad`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_stiffness-1]
                    :end-before: [end-command-line-set_joint_stiffness-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_joint_stiffness-1]
                    :end-before: [end-python-set_joint_stiffness-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_stiffness-1]
                    :end-before: [end-command-line-set_joint_stiffness-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_joint_stiffness-1]
                    :end-before: [end-python-set_joint_stiffness-1]


Set joint damping
^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_joint_damping`
      - `libiiwa_msgs/SetArray <ros.html#setarray-srv>`_
      - [0.0, 1.0]
      - unitless

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_damping-1]
                    :end-before: [end-command-line-set_joint_damping-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_joint_damping-1]
                    :end-before: [end-python-set_joint_damping-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_damping-1]
                    :end-before: [end-command-line-set_joint_damping-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_joint_damping-1]
                    :end-before: [end-python-set_joint_damping-1]

Cartesian impedance control
---------------------------

Set Cartesian stiffness
^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_stiffness`
      - `libiiwa_msgs/SetXYZABCParam <ros.html#setxyzabcparam-srv>`_
      - translational: [0.0,\ |_| \5000.0], rotational: [0.0,\ |_| \300.0], null_space: [0.0,\ |_| \Inf)
      - translational:\ |_|\ :literal:`N/m`, rotational:\ |_|\ :literal:`Nm/rad`, null_space:\ |_|\ :literal:`Nm/rad`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_stiffness-1]
                    :end-before: [end-command-line-set_cartesian_stiffness-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_stiffness-1]
                    :end-before: [end-python-set_cartesian_stiffness-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_stiffness-1]
                    :end-before: [end-command-line-set_cartesian_stiffness-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_stiffness-1]
                    :end-before: [end-python-set_cartesian_stiffness-1]

Set Cartesian damping
^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_damping`
      - `libiiwa_msgs/SetXYZABCParam <ros.html#setxyzabcparam-srv>`_
      - translational: [0.1,\ |_| \1.0], rotational: [0.1,\ |_| \1.0], null_space: [0.3,\ |_| \0.1]
      - unitless

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_damping-1]
                    :end-before: [end-command-line-set_cartesian_damping-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_damping-1]
                    :end-before: [end-python-set_cartesian_damping-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_damping-1]
                    :end-before: [end-command-line-set_cartesian_damping-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_damping-1]
                    :end-before: [end-python-set_cartesian_damping-1]

Set Cartesian additional control force
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_additional_control_force`
      - `libiiwa_msgs/SetXYZABC <ros.html#setxyzabc-srv>`_
      - \-
      - translational:\ |_|\ :literal:`N`, rotational:\ |_|\ :literal:`Nm`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_additional_control_force-1]
                    :end-before: [end-command-line-set_cartesian_additional_control_force-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_additional_control_force-1]
                    :end-before: [end-python-set_cartesian_additional_control_force-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_additional_control_force-1]
                    :end-before: [end-command-line-set_cartesian_additional_control_force-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_additional_control_force-1]
                    :end-before: [end-python-set_cartesian_additional_control_force-1]

Set Cartesian maximum control force
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_max_control_force`
      - `libiiwa_msgs/SetXYZABCParam <ros.html#setxyzabcparam-srv>`_
      - translational: [0.0,\ |_| \Inf), rotational: [0.0,\ |_| \Inf)
      - translational:\ |_|\ :literal:`N`, rotational:\ |_|\ :literal:`Nm`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_control_force-1]
                    :end-before: [end-command-line-set_cartesian_max_control_force-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_control_force-1]
                    :end-before: [end-python-set_cartesian_max_control_force-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_control_force-1]
                    :end-before: [end-command-line-set_cartesian_max_control_force-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_control_force-1]
                    :end-before: [end-python-set_cartesian_max_control_force-1]

Set Cartesian maximum velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_max_velocity`
      - `libiiwa_msgs/SetXYZABC <ros.html#setxyzabc-srv>`_
      - translational: [0.0,\ |_| \Inf), rotational: [0.0,\ |_| \Inf)
      - translational:\ |_|\ :literal:`m/s`, rotational:\ |_|\ :literal:`rad/s`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_velocity-1]
                    :end-before: [end-command-line-set_cartesian_max_velocity-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_velocity-1]
                    :end-before: [end-python-set_cartesian_max_velocity-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_velocity-1]
                    :end-before: [end-command-line-set_cartesian_max_velocity-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_velocity-1]
                    :end-before: [end-python-set_cartesian_max_velocity-1]

Set Cartesian maximum path deviation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_cartesian_max_path_deviation`
      - `libiiwa_msgs/SetXYZABC <ros.html#setxyzabc-srv>`_
      - translational: [0.0,\ |_| \Inf), rotational: [0.0,\ |_| \Inf)
      - translational:\ |_|\ :literal:`m`, rotational:\ |_|\ :literal:`radians`

Implementation details
""""""""""""""""""""""

# TODO

Example
"""""""

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_path_deviation-1]
                    :end-before: [end-command-line-set_cartesian_max_path_deviation-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_path_deviation-1]
                    :end-before: [end-python-set_cartesian_max_path_deviation-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: bash
                    :start-after: [start-command-line-set_cartesian_max_path_deviation-1]
                    :end-before: [end-command-line-set_cartesian_max_path_deviation-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_impedance.txt
                    :language: python
                    :start-after: [start-python-set_cartesian_max_path_deviation-1]
                    :end-before: [end-python-set_cartesian_max_path_deviation-1]

Cartesian sine impedance control
--------------------------------

:orange:`WILL BE INCLUDED SOON!`

.. # TODO: add support for cartesian sine impedance control
