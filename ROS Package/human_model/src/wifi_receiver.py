#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
import tf2_ros
from math import cos,sin
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

quat = Quaternion()

leg_ws = TransformStamped()
body = TransformStamped()
head = TransformStamped()
right_upper = TransformStamped()
right_lower = TransformStamped()
right_hand = TransformStamped()
left_upper = TransformStamped()
left_lower = TransformStamped()
left_hand = TransformStamped()

# DEVICE RELATED
NR_OF_DEVICES = 1

shift = [[0]*3 for _ in range(NR_OF_DEVICES)]
# up_shift = [[0]*3 for _ in range(NR_OF_DEVICES)]
# down_shift = [[0]*3 for _ in range(NR_OF_DEVICES)]

def Init():
    # DEFINED FRAME IDS
    leg_ws.header.frame_id    = "world"
    leg_ws.child_frame_id     = "leg_ws"
    body.header.frame_id    = "leg_ws"
    body.child_frame_id     = "body"
    head.header.frame_id  = "body"
    head.child_frame_id   = "head"
    right_upper.header.frame_id  = "body"
    right_upper.child_frame_id   = "right_upper"
    right_lower.header.frame_id  = "right_upper"
    right_lower.child_frame_id   = "right_lower"
    right_hand.header.frame_id  = "right_lower"
    right_hand.child_frame_id   = "right_hand"
    left_upper.header.frame_id  = "body"
    left_upper.child_frame_id   = "left_upper"
    left_lower.header.frame_id  = "left_upper"
    left_lower.child_frame_id   = "left_lower"
    left_hand.header.frame_id  = "left_lower"
    left_hand.child_frame_id   = "left_hand"
    # FIXED POSITIONS
    leg_ws.transform.rotation.z       = 0.1097783
    leg_ws.transform.rotation.w       = 0.9939561
    body.transform.translation.x  = -0.035
    body.transform.translation.z  = 1.1
    body.transform.rotation.z     = -0.1097783
    body.transform.rotation.w     = 0.9939561
    
    head.transform.translation.z  = 0.52
    right_upper.transform.translation.x  = 0.015
    right_upper.transform.translation.y  = -0.225
    right_upper.transform.translation.z  = 0.46
    right_lower.transform.translation.x  = -0.01
    right_lower.transform.translation.y  = -0.015
    right_lower.transform.translation.z  = -0.368
    right_hand.transform.translation.x  = 0.05
    right_hand.transform.translation.z  = -0.29

    left_upper.transform.translation.x  = 0.015
    left_upper.transform.translation.y  = 0.225
    left_upper.transform.translation.z  = 0.46
    left_lower.transform.translation.x  = -0.01
    left_lower.transform.translation.y  = 0.015
    left_lower.transform.translation.z  = -0.368
    left_hand.transform.translation.x  = 0.05
    left_hand.transform.translation.z  = -0.29

def ToQuat(roll, pitch, yaw):
    cr = cos(roll*0.5)
    sr = sin(roll*0.5)
    cp = cos(pitch*0.5)
    sp = sin(pitch*0.5)
    cy = cos(yaw*0.5)
    sy = sin(yaw*0.5)

    q = Quaternion()

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("angles")

def on_message(client, userdata, msg):
    br = tf2_ros.TransformBroadcaster()

    # TAKE MESSAGE, CLEAR BUFFER
    hold = str(msg.payload)
    quat_buffer = []
    euler_buffer = []

    # TIME STAMP
    leg_ws.header.stamp = rospy.Time.now()
    body.header.stamp = rospy.Time.now()
    head.header.stamp = rospy.Time.now()
    right_upper.header.stamp = rospy.Time.now()
    right_lower.header.stamp = rospy.Time.now()
    right_hand.header.stamp = rospy.Time.now()
    left_upper.header.stamp = rospy.Time.now()
    left_lower.header.stamp = rospy.Time.now()
    left_hand.header.stamp = rospy.Time.now()
    
    # PARSE ALL ROTATIONS(NOT USED ANYMORE)
    # for _ in range(2):
    #     values = hold[0:hold.find(";")]
    #     x = float(values[2:values.find(",")])
    #     values = values[values.find(",")+1:]
    #     y = float(values[2:values.find(",")])
    #     values = values[values.find(",")+1:]
    #     z = float(values[2:])
    #     hold = hold[hold.find(";")+1:]
    #     quat = ToQuat(x,y,z)
    #     buffer.append(quat)

    # PARSE 0 --------------------------------------- LEFT UPPER
    values = hold[0:hold.find(";")]
    x = float(values[2:values.find(",")])
    values = values[values.find(",")+1:]
    y = float(values[2:values.find(",")])
    values = values[values.find(",")+1:]
    z = float(values[2:])
    hold = hold[hold.find(";")+1:]
    # LIMITS SHIFT CORRECTION
    # X
    if (x-shift[0][0]) > 1:
        shift[0][0] = x - 1
    if (x-shift[0][0]) < -0.1:
        shift[0][0] = x + 0.1
    # Y
    if (y-shift[0][1]) > 0.3:
        shift[0][1] = y - 0.3
    if (y-shift[0][1]) < -0.3:
        shift[0][1] = y + 0.3
    # Z
    if (z-shift[0][2]) > 0.1:
        shift[0][2] = z - 0.1
    if (z-shift[0][2]) < -1.5:
        shift[0][2] = z + 1.5
    x_out = x - shift[0][0]
    y_out = y - shift[0][1]
    z_out = z - shift[0][2]
    # # APPEND TO ARRAY
    euler_buffer.append([-x_out,z_out,y_out])
    # euler_buffer.append([-x,z,y])
    # print(values)

    # PARSE 1 --------------------------------------- LEFT LOWER
    # values = hold[0:hold.find(";")]
    # x -= float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y -= float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z -= float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])
    # print(hold)

    # # PARSE 2 --------------------------------------- LEFT HAND
    # values = hold[0:hold.find(";")]
    # x += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z += float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])

    # # PARSE 3: RIGHT UPPER
    # values = hold[0:hold.find(";")]
    # x = float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y = float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z = float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])

    # # PARSE 4 --------------------------------------- RIGHT LOWER
    # values = hold[0:hold.find(";")]
    # x += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z += float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])

    # # PARSE 5 --------------------------------------- RIGHT HAND
    # values = hold[0:hold.find(";")]
    # x += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y += float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z += float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])

    # # PARSE 6 --------------------------------------- HEAD
    # values = hold[0:hold.find(";")]
    # x = float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # y = float(values[2:values.find(",")])
    # values = values[values.find(",")+1:]
    # z = float(values[2:])
    # hold = hold[hold.find(";")+1:]
    # euler_buffer.append([x,y,z])
    
    # EULER TO QUATERNION
    for i in range(NR_OF_DEVICES):
        quat = ToQuat(euler_buffer[i][0],euler_buffer[i][1],euler_buffer[i][2])
        quat_buffer.append(quat)

    # MAP
    # left_upper.transform.rotation = quat_buffer[0]
    # left_lower.transform.rotation = quat_buffer[1]
    # left_hand.transform.rotation = quat_buffer[2]
    right_upper.transform.rotation = quat_buffer[0]
    # right_lower.transform.rotation = quat_buffer[4]
    # right_hand.transform.rotation = quat_buffer[5]
    # head.transform.rotation = quat_buffer[6]
    # print(head.header.stamp.nsecs)

    # SEND
    br.sendTransform(leg_ws)
    br.sendTransform(body)
    # br.sendTransform(left_upper)
    # br.sendTransform(left_lower)
    # br.sendTransform(left_hand)
    br.sendTransform(right_upper)
    # br.sendTransform(right_lower)
    # br.sendTransform(right_hand)
    # # br.sendTransform(head)

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("192.168.0.101", 1883, 60)

    while not rospy.is_shutdown():
        client.disconnect()
        client.loop()

if __name__ == '__main__':
    try:
        rospy.init_node('wifi_receiver')
        Init()
        main()
    except rospy.ROSInterruptException:
        pass