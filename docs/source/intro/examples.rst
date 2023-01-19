Examples
========

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

.. raw:: html

    <hr>

Reinforcement learning
----------------------

3D reaching task (iiwa's end-effector must reach a certain target point in space). The training was done in Omniverse Isaac Gym using the `skrl <https://skrl.readthedocs.io>`_ reinforcement learning library, . The real robot control is performed through the Python, ROS and ROS2 APIs using the same reinforcement learning library. Training and evaluation is performed for both Cartesian and joint control space

.. note::
    
    Visit the **skrl** documentation, under `Real-world examples <https://skrl.readthedocs.io/en/latest/intro/examples.html#real-world-examples>`_ section, **to access the training and evaluation files** (both in simulation and in real-world)

**Implementation** (see details in the table below):

* The observation space is composed of the episode's normalized progress, the robot joints' normalized positions (:math:`q`) in the interval -1 to 1, the robot joints' velocities (:math:`\dot{q}`) affected by a random uniform scale for generalization, and the target's position in space (:math:`target_{_{XYZ}}`) with respect to the robot's base

* The action space, bounded in the range -1 to 1, consists of the following. For the joint control it's robot joints' position scaled change. For the Cartesian control it's the end-effector's position (:math:`ee_{_{XYZ}}`) scaled change

* The instantaneous reward is the negative value of the Euclidean distance (:math:`\text{d}`) between the robot end-effector and the target point position. The episode terminates when this distance is less than 0.035 meters in simulation (0.075 meters in real-world) or when the defined maximum timestep is reached

* The target position lies within a rectangular cuboid of dimensions 0.2 x 0.4 x 0.4 meters centered at (0.6, 0.0, 0.4) meters with respect to the robot's base. The robot joints' positions are drawn from an initial configuration [0º, 0º, 0º, -90º, 0º, 90º, 0º] modified with uniform random values between -7º and 7º approximately

.. list-table::
    :header-rows: 1

    * - Variable
        - Formula / value
        - Size
    * - Observation space
        - :math:`\dfrac{t}{t_{max}},\; 2 \dfrac{q - q_{min}}{q_{max} - q_{min}} - 1,\; 0.1\,\dot{q}\,U(0.5,1.5),\; target_{_{XYZ}}`
        - 18
    * - Action space (joint)
        - :math:`\dfrac{2.5}{120} \, \Delta q`
        - 7
    * - Action space (Cartesian)
        - :math:`\dfrac{1}{100} \, \Delta ee_{_{XYZ}}`
        - 3
    * - Reward
        - :math:`-\text{d}(ee_{_{XYZ}},\; target_{_{XYZ}})`
        -
    * - Episode termination
        - :math:`\text{d}(ee_{_{XYZ}},\; target_{_{XYZ}}) \le 0.035 \quad` or :math:`\quad t \ge t_{max} - 1`
        -
    * - Maximum timesteps (:math:`t_{max}`)
        - 100
        -

.. raw:: html

    <hr>

**Real-world (Python)**

.. raw:: html

    <video width="100%" controls autoplay>
        <source src="https://user-images.githubusercontent.com/22400377/212192766-9698bfba-af27-41b8-8a11-17ed3d22c020.mp4" type="video/mp4">
    </video>
    <br><br>

**Real-world (ROS/ROS2)**

.. raw:: html

    <video width="100%" controls autoplay>
        <source src="https://user-images.githubusercontent.com/22400377/212192817-12115478-e6a8-4502-b33f-b072664b1959.mp4" type="video/mp4">
    </video>
    <br><br>

**Simulation (Omniverse Isaac Gym)**

.. raw:: html

    <video width="100%" controls autoplay>
        <source src="https://user-images.githubusercontent.com/22400377/211668313-7bcbcd41-cde5-441e-abb4-82fff7616f06.mp4" type="video/mp4">
    </video>

    <img width="100%" src="https://user-images.githubusercontent.com/22400377/212194442-f6588b98-38af-4f29-92a3-3c853a7e31f4.png">

.. raw:: html

    <hr>

MoveIt in RViz
--------------

.. raw:: html

    <video width="100%" controls autoplay>
        <source src="https://user-images.githubusercontent.com/22400377/213265349-215f02fb-fb3e-4c7d-9540-c26849d8bfe1.mp4" type="video/mp4">
    </video>
