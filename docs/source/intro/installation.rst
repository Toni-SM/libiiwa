Installation
============

.. raw:: html

    <hr>
    
Prerequisites
-------------

**libiiwa** requires Python 3.6 or higher and the following libraries (they will be installed automatically):

    * `NumPy <https://numpy.org/>`_

.. raw:: html

    <hr>

Library Installation
--------------------

GitHub repository
^^^^^^^^^^^^^^^^^

Clone or download the library from its GitHub repository (https://github.com/Toni-SM/libiiwa)

    .. code-block:: bash
        
        git clone https://github.com/Toni-SM/libiiwa.git
        cd libiiwa

* **Install in editable/development mode** (links the package to its original location allowing any modifications to be reflected directly in its Python environment)

    .. code-block:: bash
        
        pip install -e .

* **Install in the current Python site-packages directory** (modifications to the code downloaded from GitHub will not be reflected in your Python environment)

    .. code-block:: bash
        
        pip install .

.. raw:: html

    <hr>

Troubleshooting
---------------

Bug detection and/or correction, feature requests and everything else are more than welcome. Come on, open a new issue!

.. centered:: https://github.com/Toni-SM/libiiwa/issues

.. Known issues
.. ------------

Changelog
---------

.. literalinclude:: ../../../CHANGELOG.md
    :language: markdown
