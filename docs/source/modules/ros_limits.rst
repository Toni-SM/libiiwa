ROS & ROS2: Velocities, accelerations and jerk limits
=====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>

Joint space
-----------

Desired relative joint velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative velocity (% of maximum velocity)

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_velocity_rel`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_velocity_rel-1]
                    :end-before: [end-command-line-desired_joint_velocity_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_velocity_rel-1]
                    :end-before: [end-python-desired_joint_velocity_rel-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_velocity_rel-1]
                    :end-before: [end-command-line-desired_joint_velocity_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_velocity_rel-1]
                    :end-before: [end-python-desired_joint_velocity_rel-1]

Desired relative joint acceleration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative acceleration (% of maximum acceleration)

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_acceleration_rel`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_acceleration_rel-1]
                    :end-before: [end-command-line-desired_joint_acceleration_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_acceleration_rel-1]
                    :end-before: [end-python-desired_joint_acceleration_rel-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_acceleration_rel-1]
                    :end-before: [end-command-line-desired_joint_acceleration_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_acceleration_rel-1]
                    :end-before: [end-python-desired_joint_acceleration_rel-1]

Desired relative joint jerk
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative jerk (% of maximum jerk)

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_jerk_rel`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_jerk_rel-1]
                    :end-before: [end-command-line-desired_joint_jerk_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_jerk_rel-1]
                    :end-before: [end-python-desired_joint_jerk_rel-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_joint_jerk_rel-1]
                    :end-before: [end-command-line-desired_joint_jerk_rel-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_joint_jerk_rel-1]
                    :end-before: [end-python-desired_joint_jerk_rel-1]

.. raw:: html
  
    <hr>

Cartesian space
---------------

Desired Cartesian velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the absolute Cartesian velocity

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_velocity`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - (0, Inf)
      - :math:`m/s`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_velocity-1]
                    :end-before: [end-command-line-desired_cartesian_velocity-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_velocity-1]
                    :end-before: [end-python-desired_cartesian_velocity-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_velocity-1]
                    :end-before: [end-command-line-desired_cartesian_velocity-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_velocity-1]
                    :end-before: [end-python-desired_cartesian_velocity-1]

Desired Cartesian acceleration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the absolute Cartesian acceleration

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_acceleration`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - (0, Inf)
      - :math:`m/s^2`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_acceleration-1]
                    :end-before: [end-command-line-desired_cartesian_acceleration-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_acceleration-1]
                    :end-before: [end-python-desired_cartesian_acceleration-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_acceleration-1]
                    :end-before: [end-command-line-desired_cartesian_acceleration-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_acceleration-1]
                    :end-before: [end-python-desired_cartesian_acceleration-1]

Desired Cartesian jerk
^^^^^^^^^^^^^^^^^^^^^^

Define the absolute Cartesian jerk

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_jerk`
      - `libiiwa_msgs/SetNumber <ros.html#setnumber-srv>`_
      - (0, Inf)
      - :math:`m/s^3`

Example:

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_jerk-1]
                    :end-before: [end-command-line-desired_cartesian_jerk-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_jerk-1]
                    :end-before: [end-python-desired_cartesian_jerk-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: bash
                    :start-after: [start-command-line-desired_cartesian_jerk-1]
                    :end-before: [end-command-line-desired_cartesian_jerk-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_limits.txt
                    :language: python
                    :start-after: [start-python-desired_cartesian_jerk-1]
                    :end-before: [end-python-desired_cartesian_jerk-1]
