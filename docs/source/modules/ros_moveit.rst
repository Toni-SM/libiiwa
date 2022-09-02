MoveIt support
==============

.. raw:: html
  
    <hr>

This library implements the `FollowJointTrajectory <http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html>`_ action server (actionlib) low level controller used typically by Moveit to command motions via the JointTrajectoryController

For the FollowJointTrajectory controller interface, the communication will take place under the name :literal:`/controller_name/action_namespace` configurable from the launch files


.. list-table:: Launch parameters for MoveIt support
    :header-rows: 1

    * - Launch parameter
      - Description
      - Type
      - Default value
    * - :literal:`controller_name`
      - The name of the controller
      - :literal:`str`
      - :literal:`"iiwa_controller"`
    * - :literal:`action_namespace`
      - The action namespace for the controller
      - :literal:`str`
      - :literal:`"follow_joint_trajectory"`
    * - :literal:`follow_all_trajectory`
      - Whether to follow the whole planned trajectory or go directly to the goal by skipping the intermediate trajectory points
      - :literal:`bool`
      - :literal:`"true"`

.. tabs::

    .. group-tab:: ROS

        **Launch file:** :literal:`libiiwa_ros/launch/default.launch`

        .. literalinclude:: ../../../ros/src/libiiwa_ros/launch/default.launch
            :language: xml
            :emphasize-lines: 5-7

    .. group-tab:: ROS2

        **Launch file:** :literal:`libiiwa_ros2/launch/default.py`

        .. literalinclude:: ../../../ros2/src/libiiwa_ros2/launch/default.py
            :language: python
            :emphasize-lines: 15-20
