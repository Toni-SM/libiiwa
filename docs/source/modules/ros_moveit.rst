MoveIt support
==============

.. |_| unicode:: 0xA0 
    :trim:

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>

This library implements the `FollowJointTrajectory <http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html>`_ action server (actionlib) low level controller used typically by Moveit to command motions via the JointTrajectoryController

For the FollowJointTrajectory controller interface, the communication will take place under the name :literal:`/controller_name/action_namespace` configurable from the launch files

Launch parameters
-----------------

.. list-table:: Launch parameters for MoveIt support
    :header-rows: 1

    * - Launch parameter
      - Description
      - Type
      - Default value
    * - :literal:`controller_name`
      - The\ |_| \name\ |_| \of\ |_| \the\ |_| \controller
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
    * - :literal:`trajectory_update_threshold`
      - Threshold used to progressively traverse trajectory points (see details below)
      - :literal:`double`
      - :literal:`0.5`

.. tabs::

    .. group-tab:: ROS

        **Launch file:** :literal:`libiiwa_ros/launch/default.launch`

        .. literalinclude:: ../../../ros/src/libiiwa_ros/launch/default.launch
            :language: xml
            :emphasize-lines: 5-8

    .. group-tab:: ROS2

        **Launch file:** :literal:`libiiwa_ros2/launch/default.py`

        .. literalinclude:: ../../../ros2/src/libiiwa_ros2/launch/default.py
            :language: python
            :emphasize-lines: 15-20

Trajectory execution implementation 
-----------------------------------

.. threshold  0.005   rel vel 0.02
.. # threshold  0.1     rel vel 0.25
.. # threshold  0.5     rel vel 0.5
.. # threshold =0.75     rel vel 0.75
.. # threshold =1.25     rel vel 1.0


:literal:`Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 1.000000 seconds). Stopping trajectory.`

Edit the MoveIt configuration package :literal:`launch/trajectory_execution.launch.xml`

* Disable execution duration monitoring

  .. code-block:: xml
      :emphasize-lines: 4
      
      <param name="trajectory_execution/execution_duration_monitoring" value="false" />

* Adjust the execution duration scaling (e.g. 5)

  .. code-block:: xml
      :emphasize-lines: 4
      
      <param name="trajectory_execution/allowed_execution_duration_scaling" value="5.0"/>
