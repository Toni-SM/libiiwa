ROS & ROS2: Conditions
======================

Conditions make it possible to monitor the robot control and trigger specific reactions (termination of a running motion) if definable limits are exceeded or not reached

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Reset conditions
----------------

Reset all conditions

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
    * - :literal:`/iiwa/reset_conditions`
      - `std_srvs/Empty <http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html>`_

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-reset_conditions-1]
                    :end-before: [end-command-line-reset_conditions-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: python
                    :start-after: [start-python-reset_conditions-1]
                    :end-before: [end-python-reset_conditions-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-reset_conditions-1]
                    :end-before: [end-command-line-reset_conditions-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: python
                    :start-after: [start-python-reset_conditions-1]
                    :end-before: [end-python-reset_conditions-1]

Force conditions
----------------

Define the force condition (threshold and tolerance) for each Cartesian axis

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Units
    * - :literal:`/iiwa/set_force_condition`
      - `libiiwa_msgs/SetArray <ros.html#setarray-srv>`_
      - # TODO

Implementation details
^^^^^^^^^^^^^^^^^^^^^^

.. note::

    The current implementation links each defined condition by a logic OR operation

# TODO

Example
^^^^^^^

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-set_force_condition-1]
                    :end-before: [end-command-line-set_force_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: python
                    :start-after: [start-python-set_force_condition-1]
                    :end-before: [end-python-set_force_condition-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-set_force_condition-1]
                    :end-before: [end-command-line-set_force_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: python
                    :start-after: [start-python-set_force_condition-1]
                    :end-before: [end-python-set_force_condition-1]

Joint torque conditions
-----------------------

Define the joint torque condition (lower and upper limits) for each joint axis

.. list-table::
    :header-rows: 1

    * - Default service name
      - Message type (srv)
      - Units
    * - :literal:`/iiwa/set_joint_torque_condition`
      - `libiiwa_msgs/SetArray <ros.html#setarray-srv>`_
      - # TODO

Implementation details
^^^^^^^^^^^^^^^^^^^^^^

.. note::

    The current implementation links each defined condition by a logic OR operation

# TODO

Example
^^^^^^^

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_torque_condition-1]
                    :end-before: [end-command-line-set_joint_torque_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_conditions.txt
                    :language: python
                    :start-after: [start-python-set_joint_torque_condition-1]
                    :end-before: [end-python-set_joint_torque_condition-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: bash
                    :start-after: [start-command-line-set_joint_torque_condition-1]
                    :end-before: [end-command-line-set_joint_torque_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_conditions.txt
                    :language: python
                    :start-after: [start-python-set_joint_torque_condition-1]
                    :end-before: [end-python-set_joint_torque_condition-1]
