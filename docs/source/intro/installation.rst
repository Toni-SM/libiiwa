Installation
============

.. raw:: html

    <hr><hr>

Requirements
------------

**Hardware:**

- `KUKA LBR iiwa <https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa>`_ cobot
- `KUKA Sunrise Cabinet <https://www.kuka.com/en-us/products/robotics-systems/robot-controllers/kuka-sunrise-cabinet>`_ (controller for KUKA's LBR iiwa cobot)
- External workstation (Linux or Windows)

**Software:**

- `KUKA Sunrise.Workbench <https://my.kuka.com/s/category/software/engineering-for-robot-systems/sunrise-workbench-series/0ZG1i000000XaaaGAC>`_ for creating and synchronizing Sunrise projects
- `KUKA Sunrise.Servoing <https://my.kuka.com/s/category/software/system-software-expansion/kuka-sunrise-extensions-series/product-family-kuka-sunriseservoing/0ZG1i000000XapCGAS?language=en_US>`_ extension

.. raw:: html

    <hr><hr>

Installation and setup
----------------------

The installation and setup is divided into two parts:

- The **Sunrise project** to be installed in the KUKA Sunrise Cabinet (programmed in JAVA) 
- The **external packages** that communicates with the Cabinet via TCP/IP

.. raw:: html

    <hr>

Sunrise project
^^^^^^^^^^^^^^^

1. In the Sunrise.Workbench, create a new project or load the existing project from the controller (default IP: :literal:`172.31.1.147`)

2. Open the :literal:`StationSetup.cat` file and enable **SmartServo** extensions (in the :literal:`Software` tab)

.. image:: ../_static/imgs/install-java-1.png
    :width: 100%
    :align: center
    :alt: Enable SmartServo

3. Copy library files (:literal:`.java` files) from :literal:`libiiwa/java/src/application` to :literal:`<sunrise_project>/src/application` folder

.. image:: ../_static/imgs/install-java-2.png
    :width: 35%
    :align: center
    :alt: Copy files

4. Edit the ProcessData configuration (:literal:`<sunrise_project>/src/RoboticsAPI.data.xml` file) and overwrite the content with the following configuration

.. tabs::

    .. tab:: Common environment

        .. code-block:: xml

                <?xml version="1.0" encoding="UTF-8" standalone="no"?>
                <RoboticsAPIData version="3">
                    <world>
                        <gravitation x="0.0" y="0.0" z="9.81"/>
                    </world>
                    <objectTemplates>
                    </objectTemplates>
                    <processDataContainer>
                        <processData dataType="java.lang.String" defaultValue="172.31.1.25" displayName="Controller: IP Address" editableOnHmi="true" id="controller_ip" value="172.31.1.25"/>
                        <processData dataType="java.lang.Integer" defaultValue="12225" displayName="Controller: Port" editableOnHmi="true" id="controller_port" max="65535" min="1024" value="12225"/>
                        <processData dataType="java.lang.Boolean" defaultValue="false" displayName="Communication: Use double precision" editableOnHmi="true" id="communication_double_precision" value="false"/>
                        <processData dataType="java.lang.Boolean" defaultValue="false" displayName="Enable verbose log" editableOnHmi="true" id="verbose" value="false"/>
                    </processDataContainer>
                </RoboticsAPIData>

.. raw:: html

    <br>

.. list-table::
    :header-rows: 1

    * - Data
      - Default
      - Description
    * - Controller: IP Address
      - :literal:`172.31.1.25`
      - External control workstation IP address
    * - Controller: Port
      - :literal:`12225`
      - External control workstation port
    * - Communication: Use double precision
      - :literal:`false`
      - Whether to use double precision (64 bits) in both the request and the response. (Default: float precision (32 bits))
    * - Enable verbose log
      - :literal:`false`
      - Whether to display information about received commands or any other data on the smartHMI. **The excessive use of the message display could degrade the application performance and the smartHMI operation**

5. Install the :literal:`StationSetup.cat` (wait for Cabinet reset) and synchronize the project

.. image:: ../_static/imgs/install-java-3.png
    :width: 100%
    :align: center
    :alt: Install StationSetup

.. raw:: html

    <hr>

External packages
^^^^^^^^^^^^^^^^^

Python
""""""

ROS/ROS2
""""""""

.. raw:: html

    <hr><hr>

Troubleshooting
---------------

Bug detection and/or correction, feature requests and everything else are more than welcome. Come on, open a new issue!

.. centered:: https://github.com/Toni-SM/libiiwa/issues

.. Known issues
.. ------------

.. raw:: html

    <hr><hr>

Changelog
---------

.. literalinclude:: ../../../CHANGELOG.md
    :language: markdown
