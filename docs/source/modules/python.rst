Class
=====

.. |_| unicode:: 0xA0 
    :trim:

.. contents:: Table of Contents
   :depth: 3
   :local:
   :backlinks: none

.. raw:: html
  
    <hr>

Python class
------------

Basic usage
^^^^^^^^^^^

.. tabs::

    .. tab:: Default instantiation

        .. code-block:: python

            # import the main class
            import libiiwa

            # init robot interface
            iiwa = libiiwa.LibIiwa()

.. note::

    After the :literal:`LibIiwa` class is instantiated, the program execution is blocked until the Java library installed in the KUKA Sunrise Cabinet is ready to be launched via the smartHMI.

Class constructor
^^^^^^^^^^^^^^^^^

.. autoclass:: libiiwa.LibIiwa

    .. automethod:: __init__
