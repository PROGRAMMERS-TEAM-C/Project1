#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray

sensor = Int32MultiArray()
sensor.data = [0, 0, 0, 0, 0, 0, 0, 0]

def callback(msg):
    print(msg.data)
    
    global sensor
    sensor.data = msg.data
    

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():
    # 왼쪽으로 치우쳐졌으면
    if sensor.data[0] < sensor.data[2]:
    # 오른쪽으로 핸들 꺾기
        xycar_msg.data = [50, 25]
    # 오른쪽으로 치우쳐졌으면
    if sensor.data[0] > sensor.data[2]:
    # 왼쪽으로 핸들 꺾기
        xycar_msg.data = [-50, 25]
    # 가운데로 가고 있으면
    if sensor.data[0] == sensor.data[2]:
    # 핸들 가운데로 놔두기
        xycar_msg.data = [0, 25]

    motor_pub.publish(xycar_msg)
