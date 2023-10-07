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
      - :literal:`true`
    * - :literal:`trajectory_update_threshold`
      - Threshold used to progressively traverse trajectory points (see details below)
      - :literal:`double`
      - :literal:`0.5`

.. tabs::

    .. group-tab:: ROS

        **Launch file:** :literal:`libiiwa_ros/launch/default.launch`

        .. literalinclude:: ../../../ros/src/libiiwa_ros/launch/default.launch
            :language: xml
            :emphasize-lines: 3-6

    .. group-tab:: ROS2

        :maroon:`The implementation will be included in upcoming releases`. Open a new `discussion <https://github.com/Toni-SM/libiiwa/discussions>`_ if you need to use this functionality ahead of time.

        .. # TODO: add support for MoveIt in ROS2

        **Launch file:** :literal:`libiiwa_ros2/launch/default.py`

        .. literalinclude:: ../../../ros2/src/libiiwa_ros2/launch/default.py
            :language: python
            :emphasize-lines: 15-26

Trajectory execution implementation
-----------------------------------

.. threshold  0.005   rel vel 0.02
.. # threshold  0.1     rel vel 0.25
.. # threshold  0.5     rel vel 0.5
.. # threshold =0.75     rel vel 0.75
.. # threshold =1.25     rel vel 1.0

The execution of the sequence of trajectories parameterized in the FollowJointTrajectory actionlib is updated according to the following formula:

:math:`\sum_{j}^{J=7} || q_j - q_{j_{trajectory}} || <=` :guilabel:`trajectory_update_threshold`

Where :math:`q` is the position of the joint of the robot

The execution is sensitive to different velocity, acceleration, and jerk values. Adjust the :guilabel:`trajectory_update_threshold` parameter (in launch file) to control the behavior of the trajectory execution (in Servoing mode) in the following way:

* intermittent trajectory: increase the parameter to update the target trajectory more quickly
* missing trajectories: decrease the parameter to update the target trajectory slower

Troubleshooting
^^^^^^^^^^^^^^^

:literal:`Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 1.000000 seconds). Stopping trajectory.`

Edit the MoveIt configuration package :literal:`launch/trajectory_execution.launch.xml`

* Disable execution duration monitoring

    .. code-block:: xml
        
        <param name="trajectory_execution/execution_duration_monitoring" value="false" />

* Adjust the execution duration scaling (e.g. 5)

    .. code-block:: xml
        
        <param name="trajectory_execution/allowed_execution_duration_scaling" value="5.0"/>
