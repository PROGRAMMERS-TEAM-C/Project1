#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 필요한 import문 정의
import rospy
import numpy as np
import cv2, random, math, time

# 전역변수
Width = 640		# 원본 이미지의 Width
Height = 480		# 원본 이미지의 Height
Offset = 350		# lpos, rpos의 Y 좌표
Ofs_threshold = 30	# ROI 나눠줄 y 좌표 threshold
pre_lpos = 100		# 이전의 lpos
pre_rpos = 570		# 이전의 rpos


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2
    # 왼쪽 차선 표시 사각형
    cv2.rectangle(img, (lpos - 5, offset - 5),
                       (lpos + 5, offset + 5),
                       (0, 255, 0), 2)
    # 오른쪽 차선 표시 사각형
    cv2.rectangle(img, (rpos - 5, offset - 5),
                       (rpos + 5, offset + 5),
                       (0, 255, 0), 2)
    # 차선 가운데 지점 표시 사각형
    cv2.rectangle(img, (center - 5, offset - 5),
                       (center + 5, offset + 5),
                       (0, 255, 0), 2)
    # 화면 가운데 지점 표시 사각형
    cv2.rectangle(img, (Width/2 - 5, offset - 5),
                       (Width/2 + 5, offset + 5),
                       (0, 0, 255), 2)
    return img

# Gaussian Blur
def gaussain_blur(img, kernel_size, sigmaX=0):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), sigmaX)

# Canny Edge
def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

# Hough Transform
def hough_transform(img, rst_img, rho, theta, threshold, minLineLength, maxLineGap):
    global Offset
    global Ofs_threshold

    roi_lines = cv2.HoughLinesP(img, rho, theta, threshold, minLineLength, maxLineGap)

    # ROI로 나눴기 때문에 offset만큼 y값 추가
    offset_temp = np.array([[[0, Offset-Ofs_threshold, 0, Offset-Ofs_threshold]]], np.int32)
    offset_lines = roi_lines + offset_temp

    # line 그리기(파란색)
    for line in offset_lines:
        cv2.line(rst_img, (line[0][0], line[0][1]), (line[0][2], line[0][3]), [255, 0, 0], 2)

    return offset_lines, rst_img

# lpos, rpos
def get_pos(lines):
    global Width, Offset
    global pre_lpos
    global pre_rpos

    slopes, slp_lines, poses = [], [], []
    lpos, rpos = 0, Width
    # lines 차원 축소
    lines = np.squeeze(lines)

    for line in lines:
        # line이 ndarray으로 나오지 않는 경우 continue
        if str(type(line)) != "<type 'numpy.ndarray'>":
            continue

        x1, y1, x2, y2 = line

        # 분모가 0이면 기울기는 0
        if x2-x1 == 0:
            slope = 0
        else: 
            slope = float(y2-y1) / float(x2-x1)
        # y2-y1:x2-x1 = Offset-y1:pos-x1
        pos = (Offset - y1) * slope + x1

        slopes.append(slope)
        slp_lines.append(line)
        poses.append(pos)

    lpos_list, rpos_list = [], []

    for i in range(len(slopes)):
        slope = slopes[i]
        x1, y1, x2, y2 = slp_lines[i]
        	
        # 기울기가 음수 & 중심 - 80(중앙선을 보지 않기 위해서) & 이전의 lpos와의 차가 30이 넘지 않는 poses[i]
        if (slope < 0) and (x2 < Width/2 - 80) and (abs(poses[i] - pre_lpos) < 30):
            #llines.append(slp_lines[i].tolist())
            lpos_list.append(poses[i])    

        # 기울기가 양수 & 중심 + 80(중앙선을 보지 않기 위해서) & 이전의 rpos와의 차가 30이 넘지 않는 poses[i]
        elif (slope > 0) and (x1 > Width/2 + 80) and (abs(poses[i] - pre_rpos) < 30):
            #rlines.append(slp_lines[i].tolist()
            rpos_list.append(poses[i])            
  
    lpos_list = np.array(lpos_list)
    rpos_list = np.array(rpos_list)

    # lpos_list의 원소 개수로 조건 지정
    if lpos_list is not None:
	if len(lpos_list) > 1:
            lpos = int((np.max(lpos_list) + np.min(lpos_list))/2)
        elif len(lpos_list) == 1:
            lpos = int((lpos_list[0] + pre_lpos)/2)
        elif len(lpos_list) == 0:
            lpos = 0
    
    # rpos_list의 원소 개수로 조건 지정
    if rpos_list is not None:
	if len(rpos_list) > 1:
    	    rpos = int((np.max(rpos_list) + np.min(rpos_list))/2)
        elif len(rpos_list) == 1:
            rpos = int((rpos_list[0]+pre_rpos)/2)
        elif len(rpos_list) == 0:
	    rpos = Width
            
    #print("base_value : {}, lpos_list : {}ls, final_value : {}".format(pre_lpos, lpos_list, lpos))
    #print("base_value : {}, rpos_list : {}, final_value : {}".format(pre_rpos, rpos_list, rpos))
    #print("----------")

    pre_lpos = lpos
    pre_rpos = rpos
    
    #print(lpos, rpos)
    return lpos, rpos

# You are to find "left and light position" of road lanes
def process_image(frame):
    global Offset
    global Ofs_threshold
    
    # ROI
    roi_img = frame[Offset-Ofs_threshold:Offset+Ofs_threshold, ]
    
    # 이진화
    gray_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)

    # Gaussian Blur
    blur_img = gaussain_blur(gray_img, 5)

    # Canny Edge
    canny_img = canny(blur_img, 80, 100)

    # Hough Transform
    hough_img = frame.copy()
    lines, hough_img = hough_transform(canny_img, hough_img, rho=1, theta=np.pi/180, threshold=10, minLineLength=0, maxLineGap=0)

    # lpos, rpos
    lpos, rpos = get_pos(lines)

    # draw rectangle
    result_img = hough_img.copy()
    result_img = draw_rectangle(result_img, lpos, rpos, offset=Offset)

#    cv2.imshow('roi_img', roi_img)
#    cv2.imshow('gray_img', gray_img)
#    cv2.imshow('canny_img', canny_img)
#    cv2.imshow('hough_img', hough_img)
#    cv2.imshow('result_img', result_img)
    
    return (lpos, rpos), result_img

# draw_steer
def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)

# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':
    cap = cv2.VideoCapture('kmu_track.mkv')
    time.sleep(3)

    while not rospy.is_shutdown():
        ret, image = cap.read()
        pos, frame = process_image(image)

	# center(두 차선의 중간 지점), angle(차선 중심과 화면 중앙 픽셀 차이), steer_angle 계산
	# 조향(steer_angle)범위는 좌우20도, 각도(angle)범위는 좌우50도, 조향범위 대 각도범위의 비 20:50의 비율 = 20/50 = 0.4
	center = (pos[0] + pos[1]) // 2
	angle = Width/2 - center
        steer_angle = angle * 0.4	# 마이너스일 때 우회전, 플러스일 때 좌회전
#	print("{} -> {}".format(angle, steer_angle))
        draw_steer(frame, steer_angle)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break
