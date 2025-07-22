# bota_sensone

Drivers for the Bota SensONE Serial and EtherCAT sensors, for the Nakama Robotics Lab

This is a own build ros2 node  within a docker container for the Senseone ethercat sensor.
The base image is based on ros2 humble and the node is coppied into the container and then build.
This node is based on their python software <https://gitlab.com/botasys/python_interface>.

inside bota_ethercat/bota_ethercat/bota_ethercat_node.py some settings can be changed:
    # temperature compensation
    # IMU active
    # FIR disable
    # FAST enable
    # CHOP enable
    # Sinc filter size

Before running check if the your ethernet port is also 'enp60s0'. tHis can be done by running the find_ethernet_adapters.py

For running the script for first time (or after making alterations) use: 'docker compose up --build'
Otherwise : 'docker compose up' suffies.
