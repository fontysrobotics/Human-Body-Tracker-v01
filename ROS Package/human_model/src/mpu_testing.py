#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
from math import cos,sin
from geometry_msgs.msg import Vector3

gyro_pub = rospy.Publisher('gyro_angles', Vector3, queue_size=100)
acc_pub = rospy.Publisher('acc_angles', Vector3, queue_size=100)
combined_pub = rospy.Publisher('combined_angles', Vector3, queue_size=100)

gyro_angles = Vector3()
acc_angles = Vector3()
combined_angles = Vector3()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("angles")

def on_message(client, userdata, msg):
    # TAKE MESSAGE, CLEAR BUFFER
    hold = str(msg.payload)
    
    # PARSE ROTATIONS
    msg1 = hold[hold.find("gyro"):hold.find("acc")]
    gyro_angles.x = float(msg1[msg1.find("X=")+2:msg1.find("Y=")-1])
    gyro_angles.y = float(msg1[msg1.find("Y=")+2:msg1.find("Z=")-1])
    gyro_angles.z = float(msg1[msg1.find("Z=")+2:msg1.find(";")])
    # print(gyro_angles)
    
    msg2 = hold[hold.find("acc"):hold.find("combined")]
    acc_angles.x = float(msg2[msg2.find("X=")+2:msg2.find("Y=")-1])
    acc_angles.y = float(msg2[msg2.find("Y=")+2:msg2.find("Z=")-1])
    acc_angles.z = float(msg2[msg2.find("Z=")+2:msg2.find(";")])
    # print(gyro_angles)

    msg3 = hold[hold.find("combined"):]
    combined_angles.x = float(msg3[msg3.find("X=")+2:msg3.find("Y=")-1])
    combined_angles.y = float(msg3[msg3.find("Y=")+2:msg3.find("Z=")-1])
    combined_angles.z = float(msg3[msg3.find("Z=")+2:msg3.find(";")])
    # print(combined_angles)
    
    gyro_pub.publish(gyro_angles)
    acc_pub.publish(acc_angles)
    combined_pub.publish(combined_angles)

if __name__ == '__main__':
    try:
        rospy.init_node('wifi_receiver')
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect("192.168.0.101", 1883, 60)

        while not rospy.is_shutdown():
            client.loop()

    except rospy.ROSInterruptException:
        pass