# Bota SensONE drivers

Drivers for the Bota SensONE Serial and EtherCAT sensors, for the Nakama Robotics Lab

## Bota EtherCAT

Package `bota_ethercat/`

This is an own built ROS 2 node, within a docker container for the SensONE EtherCAT sensor.
The base image is based on ROS 2 Humble and the node is copied into the container and then build.
This node is based on their python software <https://gitlab.com/botasys/python_interface>.

Inside `bota_ethercat/bota_ethercat/bota_ethercat_node.py` some settings can be changed:

- temperature compensation
- IMU active
- FIR disable
- FAST enable
- CHOP enable
- SINC filter size

Before running check if your Ethernet port is also 'enp60s0'. This can be done by running the `find_ethernet_adapters.py`

For running the script for first time (or after making alterations) use: `docker compose up --build`
Otherwise : `docker compose up` suffices.
