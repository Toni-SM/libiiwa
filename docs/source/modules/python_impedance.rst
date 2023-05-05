Impedance control
=================

Impedance control allows implementation of compliant robot behavior

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>
    <style>.section > h3 { display: none; }</style>

Joint impedance control
-----------------------

set_joint_stiffness
^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_joint_stiffness

set_joint_damping
^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_joint_damping

Cartesian impedance control
---------------------------

set_cartesian_stiffness
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_stiffness

set_cartesian_damping
^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_damping

set_cartesian_additional_control_force
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
.. automethod:: libiiwa.LibIiwa.set_cartesian_additional_control_force

set_cartesian_max_control_force
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
.. automethod:: libiiwa.LibIiwa.set_cartesian_max_control_force

set_cartesian_max_velocity
^^^^^^^^^^^^^^^^^^^^^^^^^^
    
.. automethod:: libiiwa.LibIiwa.set_cartesian_max_velocity

set_cartesian_max_path_deviation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
.. automethod:: libiiwa.LibIiwa.set_cartesian_max_path_deviation

Cartesian sine impedance control
--------------------------------

.. autoclass:: libiiwa.CartesianDOF
    :undoc-members:
    :show-inheritance:
    :members:

.. autoclass:: libiiwa.CartesianPlane
    :undoc-members:
    :show-inheritance:
    :members:

overlay_desired_force
^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.overlay_desired_force

overlay_sine_pattern
^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.overlay_sine_pattern

overlay_lissajous_pattern
^^^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.overlay_lissajous_pattern

overlay_spiral_pattern
^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.overlay_spiral_pattern

set_cartesian_amplitude
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_amplitude

set_cartesian_frequency
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_frequency

set_cartesian_phase
^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_phase

set_cartesian_bias
^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_bias

set_cartesian_force_limit
^^^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_force_limit

set_cartesian_position_limit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_position_limit

set_cartesian_total_time
^^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_total_time

set_cartesian_rise_time
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_rise_time

set_cartesian_hold_time
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_hold_time

set_cartesian_fall_time
^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_fall_time

set_cartesian_stay_active_until_pattern_finished
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automethod:: libiiwa.LibIiwa.set_cartesian_stay_active_until_pattern_finished
