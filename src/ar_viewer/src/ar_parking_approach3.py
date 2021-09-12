#! /usr/bin/env python
# -*- coding: utf-8 -*-
#### DX값과 DY값을 arctan계산하여 angle값 구한 버전
import rospy, math, time
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":1.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

roll, pitch, yaw = 0, 0, 0

def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        
def back_drive(angle, cnt): #자동차를 후진시키는 함수, 속도를 -10으로 세팅하여 모터제어 토픽을 발행한다.
    global xycar_msg, motor_pub
    for cnt in range(cnt): #cnt회수만큼 토픽을 발행한다.
        xycar_msg.data = [angle, -10]
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        print(angle)
    for cnt in range(cnt): #cnt회수의 1/3만큼 토픽을 발행한다. 회전 방향은 반대로 한다.
        xycar_msg.data = [-angle, -10]
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        print(angle)

rospy.init_node('ar_drive')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )

xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))

    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    angle = math.degrees(np.arctan(arData["DX"] / arData["DY"])) * 2.0
    print(arData["DX"], arData["DY"])
    
    if(arData["DY"] > 150): #DY값이 150 이상이면 멀리 있는 것이므로 속도를 빠르게 -> 30
        speed = 30
    elif(arData["DY"] > 100): #DY값이 100~150 사이면 속도를 조금 낮추기 -> 20
        speed = 20
    elif(arData["DY"] > 70): #DY값이 70~100 사이면 속도를 더 낮추기 -> 10
        speed = 10
        if (round(yaw, 1) > 5.0 or round(yaw, 1) < -5.0):
            t_end = time.time() + 2.0
            while time.time() < t_end:
                speed = -25
                angle = round(yaw, 1) * -4.0
                xycar_msg.data = [angle, speed]
                motor_pub.publish(xycar_msg)
                
            t_end = time.time() + 1.0
            while time.time() < t_end:
                speed = -25
                angle = round(yaw, 1) * 4.0
                xycar_msg.data = [angle, speed]
                motor_pub.publish(xycar_msg)
    else:
        speed = 0
    
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

cv2.destroyAllWindows()



