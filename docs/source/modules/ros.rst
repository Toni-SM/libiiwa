ROS & ROS2
==========

.. |_| unicode:: 0xA0 
    :trim:

.. contents:: Table of Contents
   :depth: 3
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

            $ roslaunch libiiwa_ros default.launch

    .. group-tab:: ROS2

        .. code-block:: bash

            $ ros2 launch libiiwa_ros2 default.py

.. note::

    After launching the node, program execution is blocked until the Java library installed in the KUKA Sunrise Cabinet is executed via the smartHMI.

Launch parameters
^^^^^^^^^^^^^^^^^

.. list-table:: Launch parameters
    :header-rows: 1

    * - Launch parameter
      - Description
      - Type
      - Default value
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
            :emphasize-lines: 9-10, 12, 15

    .. group-tab:: ROS2

        **Launch file:** :literal:`libiiwa_ros2/launch/default.py`

        .. literalinclude:: ../../../ros2/src/libiiwa_ros2/launch/default.py
            :language: python
            :emphasize-lines: 29-34, 36-38, 41-43

ROS/ROS2 messages
-----------------

Services
^^^^^^^^

GetBool.srv
"""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/GetBool.srv
    :language: yaml

GetError.srv
""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/GetError.srv
    :language: yaml

GetNumber.srv
"""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/GetNumber.srv
    :language: yaml

SetArray.srv
""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/SetArray.srv
    :language: yaml

SetNumber.srv
"""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/SetNumber.srv
    :language: yaml

SetString.srv
"""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/SetString.srv
    :language: yaml

SetXYZABC.srv
"""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/SetXYZABC.srv
    :language: yaml

SetXYZABCParam.srv
""""""""""""""""""

.. literalinclude:: ../../../ros/src/libiiwa_msgs/srv/SetXYZABCParam.srv
    :language: yaml
