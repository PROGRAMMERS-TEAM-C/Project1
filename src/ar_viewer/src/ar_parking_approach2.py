#! /usr/bin/env python
# -*- coding: utf-8 -*-
### artag 거리 정보와 yaw 값 이용한 버전2
import rospy, math, time
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

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
        
    for cnt in range(cnt): #cnt회수의 1/3만큼 토픽을 발행한다. 회전 방향은 반대로 한다.
        xycar_msg.data = [-angle, -10]
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        

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

    
    if arData["DX"] >= 0: #AR태그가 오른쪽에 있을 경우
        if (yaw < 0):   #Yaw값이 음수이면 크게 우회전
            angle = 50
        elif (yaw >= 0 and yaw < 10): #yaw값이 0~10 사이인 경우 (약간 비스듬히 보이는 경우)
            if(arData["DX"] < 3): #DX값이 0~2 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] < 5): #DX 3~4이면 작게 우회전
                angle = 20
            else:
                angle = 50  #DX값이 5 이상이면 크게 우회전
                
        elif (yaw >= 10): #yaw값이 10보다 큰 경우 (많이 비스듬히 보이는 경우)
            if(arData["DX"] < 3): #DX값이 0~2 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] < 5): #DX 3~4이면 작게 우회전, 하지만 yaw값이 0~10 사이일 때보단 적게
                angle = 10
            else:
                angle = 25  #DX값이 5 이상이면 크게 우회전, 하지만 yaw값이 0~10 사이일 때보단 적게
    elif arData["DX"] < 0: #AR태그가 오른쪽에 있을 경우
        if (yaw > 0):   #Yaw값이 양수이면 크게 좌회전
            angle = -50
        elif (yaw <= 0 and yaw > -10): #yaw값이 -10~0 사이인 경우 (약간 비스듬히 보이는 경우)
            if(arData["DX"] > -3): #DX값이 -2~0 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] > -5): #DX -4~-3이면 작게 좌회전
                angle = -20
            else:
                angle = -50  #DX값이 -5 이하이면 크게 좌회전
                
        elif (yaw <= -10): #yaw값이 -10보다 작은 경우 (많이 비스듬히 보이는 경우)
            if(arData["DX"] > -3): #DX값이 -2~0 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] > -5): #DX -4~-3이면 작게 좌회전, 위 보단 작은 각도로
                angle = -10
            else:
                angle = -25  #DX값이 -5 이하이면 크게 좌회전

    if(arData["DY"] > 150): #DY값이 150 이상이면 멀리 있는 것이므로 속도를 빠르게 -> 30
        speed = 30
    elif(arData["DY"] > 100): #DY값이 100~150 사이면 속도를 조금 낮추기 -> 20
        speed = 20
    elif(arData["DY"] > 70): #DY값이 70~100 사이면 속도를 더 낮추기 -> 10
        speed = 10
        
        if (yaw > 10 or abs(arData["DX"] > 100)): #Yaw값이 10 이상이거나 DX값이 100 이상이면 후진(시계방향으로 후진)
            back_drive(-50, 45)
        elif (yaw > 10 or abs(arData["DX"] > 100)): #Yaw값이 -10 이하이거나 DX값이 100 이상이면 후진(반시계방향으로 후진)
            back_drive(50, 45)
    else:
        speed = 0
    
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

cv2.destroyAllWindows()



