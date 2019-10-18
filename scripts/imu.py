#!/usr/bin/env python


import math
import re
import serial
import struct
import time

import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu


cov_orientation = [0.00174533, 0, 0, 0, 0.00174533, 0, 0, 0, 0.00174533]
cov_angular_velocity, cov_linear_acceleration = [0.00104720, 0, 0, 0, 0.00104720, 0, 0, 0, 0.00104720], [0.0049, 0, 0, 0, 0.0049, 0, 0, 0, 0.0049]


def eul_to_qua(Eular):
    Eular_Div = [0, 0, 0]
    Eular_Div[0], Eular_Div[1], Eular_Div[2] = Eular[0]/2.0, Eular[1]/2.0, Eular[2]/2.0
    
    ca, cb, cc = math.cos(Eular_Div[0]), math.cos(Eular_Div[1]), math.cos(Eular_Div[2])
    sa, sb, sc = math.sin(Eular_Div[0]), math.sin(Eular_Div[1]), math.sin(Eular_Div[2])
    
    x = sa*cb*cc - ca*sb*sc
    y = ca*sb*cc + sa*cb*sc
    z = ca*cb*sc - sa*sb*cc
    w = ca*cb*cc + sa*sb*sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation


accCali = "\xff\xaa\x67"
feature = "UQ(.{6,6}).{3,3}UR(.{6,6}).{3,3}US(.{6,6}).{3,3}"
fmt_B, fmt_h = "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB", "<hhh"


if __name__ == "__main__":
    rospy.init_node("imu")

    port = rospy.get_param("~port", "/dev/imu")
    imuHandle = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    
    if imuHandle.isOpen():
        rospy.loginfo("SUCCESS")
    else:
        rospy.loginfo("FAILURE")

    # Accelerometer Calibration
    time.sleep(0.5)
    imuHandle.write(accCali) 
    time.sleep(0.5)

    imuPub = rospy.Publisher("imu", Imu, queue_size=10)

    while True:
        data = imuHandle.read(size=65)
        stamp = rospy.get_rostime()
        result = re.search(feature, data)

        if result:
            # check and get data
            frame = struct.unpack(fmt_B, result.group())
            sum_Q, sum_R, sum_S = 0, 0, 0
            for i in range(0, 10):
                sum_Q, sum_R, sum_S = sum_Q+frame[i], sum_R+frame[i+11], sum_S+frame[i+22]
            sum_Q, sum_R, sum_S = sum_Q&0x000000ff, sum_R&0x000000ff, sum_S&0x000000ff

            if (sum_Q==frame[10]) and (sum_R==frame[21]) and (sum_S==frame[32]):
                af, wf, ef = struct.unpack(fmt_h, result.group(1)), struct.unpack(fmt_h, result.group(2)), struct.unpack(fmt_h, result.group(3))

                af_l, wf_l, ef_l = [], [], []
                for i in range(0, 3):
                    af_l.append(af[i]/32768.0*16*-9.8), wf_l.append(wf[i]/32768.0*2000*math.pi/180), ef_l.append(ef[i]/32768.0*math.pi)
                
                
                print math.degrees(ef_l[2])

                imuMsg = Imu()
                imuMsg.header.stamp, imuMsg.header.frame_id = stamp, "base_link"
                imuMsg.orientation = eul_to_qua(ef_l)
                imuMsg.orientation_covariance = cov_orientation
                imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z = wf_l[0], wf_l[1], wf_l[2]
                imuMsg.angular_velocity_covariance = cov_angular_velocity
                imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z = af_l[0], af_l[1], af_l[2]
                imuMsg.linear_acceleration_covariance = cov_linear_acceleration

                imuPub.publish(imuMsg)

        time.sleep(0.01)

