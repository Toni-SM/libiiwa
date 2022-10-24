ROS & ROS2: Move the robot
==========================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>

Stop the robot
--------------

Stop the current motion

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
    * - :literal:`/iiwa/command/stop`
      - `std_msgs/Empty <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Empty.html>`_

.. raw:: html
  
    <hr>

Joint space
-----------

Move the robot to the specified joint position

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
      - Units
    * - :literal:`/iiwa/command/joint`
      - `sensor_msgs/JointState <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html>`_
      - radians

Implementation details
^^^^^^^^^^^^^^^^^^^^^^

    * Joint names are the same as those of the `iiwa_description <https://github.com/IFL-CAMP/iiwa_stack/tree/master/iiwa_description>`_ URDF model (``iiwa_joint_1``, ``iiwa_joint_2``, ``iiwa_joint_3``, ``iiwa_joint_4``, ``iiwa_joint_5``, ``iiwa_joint_6``, ``iiwa_joint_7``)

    * Joint names, and their respective positions, can appear in any order

    * No need to specify message name if all joint positions are specified

    * A subset of the joints can be controlled by specifying only the joint names and the corresponding positions, or by setting the other joints positions to ``nan``. In this case, the excluded joints will keep their current position

Example
^^^^^^^

Move all joints to their respective goal positions
""""""""""""""""""""""""""""""""""""""""""""""""""

* Joint names: [``iiwa_joint_1``, ``iiwa_joint_2``, ``iiwa_joint_3``, ``iiwa_joint_4``, ``iiwa_joint_5``, ``iiwa_joint_6``, ``iiwa_joint_7``]
* Joint positions: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-command-line-joint-1]
                    :end-before: [end-command-line-joint-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-python-joint-1]
                    :end-before: [end-python-joint-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-command-line-joint-1]
                    :end-before: [end-command-line-joint-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-python-joint-1]
                    :end-before: [end-python-joint-1]

Move only the specified joints to their respective goal positions
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

* Joint names: [``iiwa_joint_4``, ``iiwa_joint_6``]
* Joint positions: [-1.57, 1.57]
* Other joints will keep their current position

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-command-line-joint-2]
                    :end-before: [end-command-line-joint-2]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-python-joint-2]
                    :end-before: [end-python-joint-2]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-command-line-joint-2]
                    :end-before: [end-command-line-joint-2]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-python-joint-2]
                    :end-before: [end-python-joint-2]

.. raw:: html
  
    <hr>

Cartesian space
---------------

Move the robot to the specified Cartesian pose

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Message type (msg)
      - Units
    * - :literal:`/iiwa/command/cartesian`
      - `geometry_msgs/Pose <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html>`_
      - meters

Implementation details
^^^^^^^^^^^^^^^^^^^^^^

.. TODO

Example
^^^^^^^

Move to the specified Cartesian pose (position and orientation)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

* Cartesian position: X, Y, Z = (0.65, 0.0, 0.2)
* Cartesian orientation: x, y, z, w = (0.0, 1.0, 0.0, 0.0) :math:`\; \rightarrow \;` A, B, C = (-180.0º, 0.0º, 180.0º)

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-1]
                    :end-before: [end-command-line-cartesian-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-python-cartesian-1]
                    :end-before: [end-python-cartesian-1]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-1]
                    :end-before: [end-command-line-cartesian-1]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-python-cartesian-1]
                    :end-before: [end-python-cartesian-1]

Move to the specified Cartesian position or orientation
"""""""""""""""""""""""""""""""""""""""""""""""""""""""

* Case 1
    * Cartesian position: X, Y, Z = (0.65, 0.0, 0.3)
    * Keep the current orientation 
* Case 2
    * Move in Z-axis only: Z = 0.4
    * Keep the current position in X, Y and orientation
* Case 3
    * Cartesian orientation: x, y, z, w = (0.0, -0.7071, 0.7071, 0.0) :math:`\; \rightarrow \;` A, B, C = (90.0º, 0.0º, 180.0º)
    * Keep the current position

.. tabs::

    .. group-tab:: ROS

        .. tabs::

            .. group-tab:: Command-line tool (rostopic)

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-2]
                    :end-before: [end-command-line-cartesian-2]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros_move.txt
                    :language: bash
                    :start-after: [start-python-cartesian-2]
                    :end-before: [end-python-cartesian-2]

    .. group-tab:: ROS2

        .. tabs::

            .. group-tab:: Command-line tool (ros2 topic)

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-command-line-cartesian-2]
                    :end-before: [end-command-line-cartesian-2]

            .. group-tab:: Python

                .. literalinclude:: ../snippets/ros2_move.txt
                    :language: bash
                    :start-after: [start-python-cartesian-2]
                    :end-before: [end-python-cartesian-2]
