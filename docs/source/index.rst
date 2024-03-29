libiiwa - Scalable and Unified Multi-Control Framework for the KUKA LBR iiwa (|version|)
========================================================================================

**libiiwa** is a scalable cross-platform multi-control framework for KUKA LBR iiwa cobots. It provides an interface that not only unifies and enables control and communication through ROS and ROS2, but also allows direct control for those applications where minimum control frequency is required through a scalable, simple and well-documented Application Programming Interface (API) in Python


| **GitHub repository:** https://github.com/Toni-SM/libiiwa
| **Questions or discussions:** https://github.com/Toni-SM/libiiwa/discussions 

**Citing libiiwa:** To cite this library (created at `Mondragon Unibertsitatea <https://www.mondragon.edu/en/home>`_) use the following reference to its `article <https://ieeexplore.ieee.org/abstract/document/10039308>`_: *"A Scalable and Unified Multi-Control Framework for KUKA LBR iiwa Collaborative Robots"*

.. code-block:: bibtex

    @inproceedings{serrano2023scalable,
      title={A Scalable and Unified Multi-Control Framework for KUKA LBR iiwa Collaborative Robots},
      author={Serrano-Mu{\~n}oz, Antonio and Elguea-Aguinaco, {\'I}{\~n}igo and Chrysostomou, Dimitris and B{\o}gh, Simon and Arana-Arexolaleiba, Nestor},
      booktitle={2023 IEEE/SICE International Symposium on System Integration (SII)},
      pages={1--5},
      year={2023},
      organization={IEEE}
    }

.. raw:: html

    <hr>

User guide
----------

.. toctree::
    :maxdepth: 2

    intro/overview
    intro/installation
    intro/examples
    intro/protocol

.. raw:: html

    <hr>

.. toctree::
    :maxdepth: 1
    :caption: Python API

    modules/python
    modules/python_state_and_errors
    modules/python_command
    modules/python_limits
    modules/python_motion_and_control
    modules/python_impedance
    modules/python_conditions
    modules/python_tools
    modules/python_utils

.. toctree::
    :maxdepth: 1
    :caption: ROS & ROS2 API

    modules/ros
    modules/ros_state_and_errors
    modules/ros_command
    modules/ros_limits
    modules/ros_motion_and_control
    modules/ros_impedance
    modules/ros_conditions
    modules/ros_moveit
