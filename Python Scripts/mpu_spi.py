#!/usr/bin/env python3
import time
from math import atan2,pi,sqrt,sin
# from mpu9250_jmdev.registers import *
from own_mpu import MPU9250
from periphery import SPI
import paho.mqtt.client as mqtt

# DEVICE RELATED
NR_OF_DEVICES = 1
spi = [0]*NR_OF_DEVICES
mpu = [0]*NR_OF_DEVICES
# [[0]*3 for _ in range(NR_OF_DEVICES)] ARRAY FORM

# DATA
gyro_values = [[0]*3 for _ in range(NR_OF_DEVICES)]
acc_values = [[0]*3 for _ in range(NR_OF_DEVICES)]

# NON-FILTERED
gyro_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
acc_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
prev_gyro_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
prev_acc_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]

# FILTERED
flt_gyro_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
flt_acc_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
prev_flt_gyro_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]
prev_flt_acc_angles = [[0]*3 for _ in range(NR_OF_DEVICES)]

euler = [[0]*3 for _ in range(NR_OF_DEVICES)]

loop_rate = 0.001
c_alpha = 0.998

hpf_rc = 1
hpf_alpha = hpf_rc/(hpf_rc+loop_rate)
# hpf_alpha = 0.9999888

lpf_rc = 5
lpf_alpha = loop_rate/(lpf_rc+loop_rate)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

def concatenate(gyro_values):
    result = ""
    for i in range(len(gyro_values)):
        result += "X=" + str(gyro_values[i][0]) + ","
        result += "Y=" + str(gyro_values[i][1]) + ","
        result += "Z=" + str(gyro_values[i][2]) + ";"
    return result
    
if __name__ == "__main__":
    try:
        # OPEN DEVICE FILES AND ASSIGN TO CORRESPONDING MPU OBJECT
        for i in range(NR_OF_DEVICES):
            dev_file = "/dev/spidev1.%s"%i
            print(dev_file,": opened")
            spi[i] = SPI(dev_file,3,8000000)
            mpu[i] = MPU9250(bus=spi[i],gfs=0x02,afs=0x01) # GFS_1000

        # CALIBRATE AND CONFIGURE ALL DEVICES
        for i in range(NR_OF_DEVICES):
            mpu[i].calibrate()
            mpu[i].configure() # Apply the settings to the registers.

        # STARTUP MQTT CLIENT
        client = mqtt.Client()
        client.on_connect = on_connect
        client.connect("192.168.0.101", 1883, 60)

        # LOOP
        while True:
            # READ ALL
            for i in range(NR_OF_DEVICES):
                gyro_values[i] = mpu[i].readGyroscopeMaster()
                acc_values[i] = mpu[i].readAccelerometerMaster()
                # print("GYRO: %s"%i,gyro_values[i])
                # print("ACC: ",acc_values[i])

            # FILTER
            for i in range(NR_OF_DEVICES):
                # INTEGRATE GYRO + FILTER + UPDATE
                gyro_angles[i][0] += gyro_values[i][0]*loop_rate*pi/180
                gyro_angles[i][1] += gyro_values[i][1]*loop_rate*pi/180
                gyro_angles[i][2] += gyro_values[i][2]*loop_rate*pi/180
                
                gyro_angles[i][0] -= gyro_angles[i][1]*sin(gyro_angles[i][2]*0.000001066)
                gyro_angles[i][1] += gyro_angles[i][0]*sin(gyro_angles[i][2]*0.000001066)

                flt_gyro_angles[i][0] = hpf_alpha*(flt_gyro_angles[i][0] + gyro_angles[i][0] - prev_flt_gyro_angles[i][0])
                flt_gyro_angles[i][1] = hpf_alpha*(flt_gyro_angles[i][1] + gyro_angles[i][1] - prev_flt_gyro_angles[i][1])
                flt_gyro_angles[i][2] = hpf_alpha*(flt_gyro_angles[i][2] + gyro_angles[i][2] - prev_flt_gyro_angles[i][2])

                prev_gyro_angles[i] = gyro_angles[i]
                prev_flt_gyro_angles[i] = flt_gyro_angles[i]

                # ACC DATA + FILTER + UPDATE
                acc_angles[i][0] = atan2(acc_values[i][1],sqrt((acc_values[i][0]*acc_values[i][0])+(acc_values[i][2]*acc_values[i][2])))
                acc_angles[i][1] = atan2(-1*acc_values[i][0],sqrt((acc_values[i][1]*acc_values[i][1])+(acc_values[i][2]*acc_values[i][2])))
                
                flt_acc_angles[i][0] = acc_angles[i][0]*(1-lpf_alpha) + prev_flt_acc_angles[i][0]*lpf_alpha
                flt_acc_angles[i][1] = acc_angles[i][1]*(1-lpf_alpha) + prev_flt_acc_angles[i][1]*lpf_alpha
                
                prev_acc_angles = acc_angles
                prev_flt_acc_angles = flt_acc_angles

                # COMPLEMENTARY FILTER
                euler[i][0] = (flt_gyro_angles[i][0]*c_alpha) + (flt_acc_angles[i][0]*(1-c_alpha))
                euler[i][1] = (flt_gyro_angles[i][1]*c_alpha) + (flt_acc_angles[i][1]*(1-c_alpha))
                euler[i][2] = flt_gyro_angles[i][2]

            # PRINT
            # print("EULER ANGLES: ",gyro_angles)

            # PUBLISH + LOOP RATE
            msg = concatenate(euler)
            client.publish("angles",msg)
            time.sleep(loop_rate)

        # CLOSE DEVICE FILES AND DISCONNECT CLIENT
        client.disconnect()
        for i in range(NR_OF_DEVICES):
            spi[i].close()
    except Exception as e:
        # CLOSE DEVICE FILES
        for i in range(NR_OF_DEVICES):
            spi[i].close()
        print(e)