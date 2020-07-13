# Human-Body-Tracker-v01

## ROS Package
The **Human Model** package includes the URDF file a of human worker model. The package includes two nodes:
1. **WiFi_Receiver:** A node used to launch an MQTT client which listens over a topic from the MQTT broker. The incoming messages from the motion tracking system are parsed and published to RViz through TF messages.
1. **MPU_Testing:** The node was used throughout development, to plot multiple signals in RQT-plot. It receives messages(specific syntax) published by the mpu_testing.py script, parses the data, and publishes to ROS topics. The topics can be used to plot the data in rqt_plot.