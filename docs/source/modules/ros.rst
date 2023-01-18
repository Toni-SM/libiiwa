ROS & ROS2
==========

.. |_| unicode:: 0xA0 
    :trim:

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>

ROS/ROS2 node
-------------

Launching the node
^^^^^^^^^^^^^^^^^^

.. tabs::

    .. group-tab:: ROS

        .. code-block:: bash

            roslaunch libiiwa_ros default.launch

    .. group-tab:: ROS2

        .. code-block:: bash

            ros2 launch libiiwa_ros2 default.py

Launch parameters
^^^^^^^^^^^^^^^^^

.. list-table:: Launch parameters*
    :header-rows: 1

    * - Launch parameter
      - Description
      - Type
      - Default value
    * - :literal:`robot_name`
      - Prefix\ |_| \added\ |_| \to\ |_| \topic\ |_| \and\ |_| \service\ |_| \names (e.g. :literal:`/iiwa/...`)
      - :literal:`str`
      - :literal:`"iiwa"`
    * - :literal:`libiiwa_ip`
      - IP address of the library communication endpoint (default: all interfaces)
      - :literal:`str`
      - :literal:`"0.0.0.0"`
    * - :literal:`libiiwa_port`
      - Port of the library communication endpoint
      - :literal:`int`
      - :literal:`12225`
    * - :literal:`servo_interface`
      - Whether to enable the servo interface at node startup
      - :literal:`bool`
      - :literal:`true`
    * - :literal:`verbose`
      - Whether to print additional information
      - :literal:`bool`
      - :literal:`false`

.. note::

    Visit :doc:`MoveIt support <../modules/ros_moveit>` for the :literal:`FollowJointTrajectory` actionlib configuration

.. raw:: html

    <br>

.. tabs::

    .. group-tab:: ROS

        **Launch file:** :literal:`libiiwa_ros/launch/default.launch`

        .. literalinclude:: ../../../ros/src/libiiwa_ros/launch/default.launch
            :language: xml
            :emphasize-lines: 2, 11-12, 14, 17

    .. group-tab:: ROS2

        **Launch file:** :literal:`libiiwa_ros2/launch/default.py`

        .. literalinclude:: ../../../ros2/src/libiiwa_ros2/launch/default.py
            :language: python
            :emphasize-lines: 10-12, 29-34, 36-38, 41-43
