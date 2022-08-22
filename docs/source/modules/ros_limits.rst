ROS: Velocities, accelerations and jerk limits
==============================================

.. raw:: html
  
    <hr>

Joint space
-----------

Desired relative joint velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative velocity (% of maximum velocity)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_velocity_rel`
      - libiiwa_msgs/SetDouble
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_joint_velocity_rel-1]
            :end-before: [end-command-line-desired_joint_velocity_rel-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-python-desired_joint_velocity_rel-1]
            :end-before: [end-python-desired_joint_velocity_rel-1]

Desired relative joint acceleration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative acceleration (% of maximum acceleration)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_acceleration_rel`
      - libiiwa_msgs/SetDouble
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_joint_acceleration_rel-1]
            :end-before: [end-command-line-desired_joint_acceleration_rel-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-python-desired_joint_acceleration_rel-1]
            :end-before: [end-python-desired_joint_acceleration_rel-1]

Desired relative joint jerk
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the axis-specific relative jerk (% of maximum jerk)

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_joint_jerk_rel`
      - libiiwa_msgs/SetDouble
      - [0, 1]
      - unitless

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_joint_jerk_rel-1]
            :end-before: [end-command-line-desired_joint_jerk_rel-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
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

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_velocity`
      - libiiwa_msgs/SetDouble
      - (0, Inf)
      - :math:`m/s`

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_cartesian_velocity-1]
            :end-before: [end-command-line-desired_cartesian_velocity-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-python-desired_cartesian_velocity-1]
            :end-before: [end-python-desired_cartesian_velocity-1]

Desired Cartesian acceleration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the absolute Cartesian acceleration

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_acceleration`
      - libiiwa_msgs/SetDouble
      - (0, Inf)
      - :math:`m/s^2`

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_cartesian_acceleration-1]
            :end-before: [end-command-line-desired_cartesian_acceleration-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-python-desired_cartesian_acceleration-1]
            :end-before: [end-python-desired_cartesian_acceleration-1]

Desired Cartesian jerk
^^^^^^^^^^^^^^^^^^^^^^

Define the absolute Cartesian jerk

.. list-table::
    :header-rows: 1

    * - Default topic name
      - Service type
      - Limits
      - Units
    * - :literal:`/iiwa/set_desired_cartesian_jerk`
      - libiiwa_msgs/SetDouble
      - (0, Inf)
      - :math:`m/s^3`

Example:

.. tabs::

    .. tab:: Command-line tool (rostopic)

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-command-line-desired_cartesian_jerk-1]
            :end-before: [end-command-line-desired_cartesian_jerk-1]

    .. tab:: Python

        .. literalinclude:: ../snippets/ros_limits.txt
            :language: bash
            :start-after: [start-python-desired_cartesian_jerk-1]
            :end-before: [end-python-desired_cartesian_jerk-1]
