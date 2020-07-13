# Human-Body-Tracker-v01


## ROS Package

The **Human Model** package includes the URDF file a of human worker model. The package includes two nodes:
1. **WiFi_Receiver:** A node used to launch an MQTT client which listens over a topic from the MQTT broker. The incoming messages from the motion tracking system are parsed and published to RViz through TF messages.
1. **MPU_Testing:** The node was used throughout development, to plot multiple signals in RQT-plot. It receives messages(specific syntax) published by the mpu_testing.py script, parses the data, and publishes to ROS topics. The topics can be used to plot the data in rqt_plot.


## Python Scripts

These are the scripts that can be run on the motion tracking system. A python library for the MPU was modified to adapt reading from device files. The library was saved in a separate file from the original one, **own_mpu.py**.

1. **mpu_spi.py:** Main script of motion tracking system. Reads from the connected MPU's through device files and publishes the data over WiFi. The script publishes a string message with a specific syntax, used with the **WiFi Receiver** node.
1. **mpu_testing.py:** A script used throughout development to publish multiple signals in one message. Used with the **MPU Testing** node.


## MQTT Broker

An MQTT broker must be installed to enable the communication between the system and ROS. The IP address of the platform where the broker is installed must be written into the python scripts running on the system, as well as the ROS nodes receiving the data from the MQTT broker. If not changed, the connection will fail.