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

:maroon:`It will be included in upcoming releases`. Open a new `discussion <https://github.com/Toni-SM/libiiwa/discussions>`_ if you need to use this functionality ahead of time.

.. # TODO: add support for cartesian sine impedance control
