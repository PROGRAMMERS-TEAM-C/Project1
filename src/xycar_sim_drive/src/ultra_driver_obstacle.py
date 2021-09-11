#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math, copy #임포트
from std_msgs.msg import Int32MultiArray

def callback(msg):
    xycar_msg = Int32MultiArray()
    angle = 0
    fl = msg.data[0]
    fm = msg.data[1]
    fr = msg.data[2]
    r = msg.data[6]
    l = msg.data[7]
    if fl < 240 and fr > 110:
        xycar_msg.data = [50, 50]
    elif fl < 60 and fr < 110:
        xycar_msg.data = [50, 50]
    elif fl > 110:
        xycar_msg.data = [-50, 50]
    else:
        xycar_msg.data = [angle, 50]
    motor_pub.publish(xycar_msg)
rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
rospy.spin()
