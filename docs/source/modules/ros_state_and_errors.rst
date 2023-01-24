ROS & ROS2: State and errors
============================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Robot state
-----------

Joint positions, velocities and torques (effort)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read the axis-specific (joint) actual positions, velocities (internally calculated as the position difference in 1/100 second interval) and external acting torques (without the component resulting from the robot weight and mass inertias during motion)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
      - Units
    * - :literal:`/iiwa/state/joint_states`
      - `sensor_msgs/JointState <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html>`_
      - position (:math:`radians`), velocity (:math:`radians/s`), torque (:math:`Nm`)

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-joint-1]
                    :end-before: [end-command-line-joint-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-joint-1]
                    :end-before: [end-python-joint-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-joint-1]
                    :end-before: [end-command-line-joint-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-joint-1]
                    :end-before: [end-python-joint-1]

End-effector Cartesian pose (position and orientation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read the end-effector Cartesian position (X,Y,Z) and orientation (A,B,C)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
      - Units
    * - :literal:`/iiwa/state/end_effector_pose`
      - `geometry_msgs/Pose <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html>`_
      - position (:math:`m`), orientation as quaternion

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-1]
                    :end-before: [end-command-line-cartesian-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-cartesian-1]
                    :end-before: [end-python-cartesian-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-1]
                    :end-before: [end-command-line-cartesian-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-cartesian-1]
                    :end-before: [end-python-cartesian-1]

End-effector Cartesian forces and torques
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read the end-effector Cartesian force (X,Y,Z) and torque (A,B,C)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
      - Units
    * - :literal:`/iiwa/state/end_effector_wrench`
      - `geometry_msgs/Wrench <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html>`_
      - force (:math:`N`), torque (:math:`Nm`)

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)
              
                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-force_torque-1]
                    :end-before: [end-command-line-force_torque-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-force_torque-1]
                    :end-before: [end-python-force_torque-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-force_torque-1]
                    :end-before: [end-command-line-force_torque-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-force_torque-1]
                    :end-before: [end-python-force_torque-1]

Has fired condition
^^^^^^^^^^^^^^^^^^^

Whether motion has terminated due to a break condition

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
    * - :literal:`/iiwa/has_fired_condition`
      - `libiiwa_msgs/GetBool <ros.html#getbool-srv>`_

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)
              
                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-has_fired_condition-1]
                    :end-before: [end-command-line-has_fired_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-has_fired_condition-1]
                    :end-before: [end-python-has_fired_condition-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-has_fired_condition-1]
                    :end-before: [end-command-line-has_fired_condition-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-has_fired_condition-1]
                    :end-before: [end-python-has_fired_condition-1]

Is ready to move
^^^^^^^^^^^^^^^^

Whether the robot is ready for motion. A true value does not necessarily mean that the brakes are open and that the robot is under servo control

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
    * - :literal:`/iiwa/is_ready_to_move`
      - `libiiwa_msgs/GetBool <ros.html#getbool-srv>`_

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)
              
                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-is_ready_to_move-1]
                    :end-before: [end-command-line-is_ready_to_move-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-is_ready_to_move-1]
                    :end-before: [end-python-is_ready_to_move-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-is_ready_to_move-1]
                    :end-before: [end-command-line-is_ready_to_move-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-is_ready_to_move-1]
                    :end-before: [end-python-is_ready_to_move-1]

Has active motion
^^^^^^^^^^^^^^^^^

Whether the robot is active. It does not provide any information on whether the robot is currently in motion (a false value does not necessarily mean that the robot is stationary)

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
    * - :literal:`/iiwa/has_active_motion`
      - `libiiwa_msgs/GetBool <ros.html#getbool-srv>`_

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)
              
                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-has_active_motion-1]
                    :end-before: [end-command-line-has_active_motion-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-has_active_motion-1]
                    :end-before: [end-python-has_active_motion-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-has_active_motion-1]
                    :end-before: [end-command-line-has_active_motion-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-has_active_motion-1]
                    :end-before: [end-python-has_active_motion-1]



.. raw:: html
  
    <hr>

Errors
------

Last registered error code

.. list-table::
    :header-rows: 1

    * - Default service name
      - Service type (srv)
    * - :literal:`/iiwa/last_error`
      - `libiiwa_msgs/GetError <ros.html#geterror-srv>`_

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rosservice)
              
                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-last_error-1]
                    :end-before: [end-command-line-last_error-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-last_error-1]
                    :end-before: [end-python-last_error-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 service)

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: bash
                    :start-after: [start-command-line-last_error-1]
                    :end-before: [end-command-line-last_error-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_state_and_errors.txt
                    :language: python
                    :start-after: [start-python-last_error-1]
                    :end-before: [end-python-last_error-1]
