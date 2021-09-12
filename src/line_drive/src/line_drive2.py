#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, copy

Width = 640
Height = 480
warp_Offset = 120
lane_bin_th_l, lane_bin_th_r = 120, 120

# 트랙 동영상 읽어 들이기
cap = cv2.VideoCapture('track2.mkv')
window_title = 'camera'

warp_img_w = Width / 2 #320
warp_img_h = Height / 2 #240

warpx_margin = 20
warpy_margin = 3

# 슬라이딩 윈도우 개수
nwindows = 9
# 슬라이딩 윈도우 넓이
margin = 12
# 선을 그리기 위해 최소한 있어야 할 점의 개수
minpix = 5

lane_bin_th = 125 #145

warp_src  = np.array([
    [30, 355],  
    [0,  385],
    [Width-45, 355],
    [Width, 385]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

calibrated = True
if calibrated:
    # 자이카 카메라의 Calibration 보정값
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    # 위에서 구한 보정 행렬값을 적용하여 이미지를 반듯하게 수정하는 함수(undistort() 호출해서 이미지 수정)
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

# 변환전과 후의 4개 점 좌표를 전댈해서 이미지를 원근변환 처리하여 새로운 이미지로 만들기
def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(img):
    global warp_img_w
    global nwindows
    global margin
    global mid_margin
    global minpix
    global lane_bin_th
    global pre_left_lane_inds
    global pre_right_lane_inds
    global lane_bin_th_l, lane_bin_th_r
    
    # 이미지에서 가우시안 블러링으로 노이즈 제거
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # HLS 포맷에서 L채널을 이용
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    #lane_bin_th = L.mean() * 1.25 + 20

    lane_bin_th_l = L[:][warp_img_w/2:].mean()
    lane_bin_th_r = L[:][warp_img_w/2:].mean() * 0.95 + 35
    
    #print(lane_bin_th_l, lane_bin_th_r)

    #_, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    _, lane_l = cv2.threshold(L, lane_bin_th_l, 255, cv2.THRESH_BINARY)
    _, lane_r = cv2.threshold(L, lane_bin_th_r, 255, cv2.THRESH_BINARY)
    lane = np.concatenate((lane_l[:warp_img_w/2], lane_r[warp_img_w/2:]), axis=0)
    
    # HSV 포맷에서 L채널을 이용
    #_, _, V = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HSV))
    #_, lane = cv2.threshold(V, lane_bin_th, 255, cv2.THRESH_BINARY)

    #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    #lane_bin_th = gray.mean() * 0.45 + 90
    #print('lane_bin_th', lane_bin_th)
    #_, lane = cv2.threshold(gray, lane_bin_th, 255, cv2.THRESH_BINARY)

    # 히스토그램이란 이미지를 구성하는 픽셀 분포에 대한 그래프
    # (1) x축: 픽셀의 x 좌표값
    # (2) y축: 특정 x 좌표값을 갖는 모든 흰색 픽셀의 개수
    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)
    # x축(x좌표)을 반으로 나누어 왼쪽 차선과 오른쪽 차선을 구분하기
    midpoint = np.int(histogram.shape[0]/2)
    # 왼쪽 절반 구역에서 흰색 픽셀의 개수가 가장 많은 위치를 슬라이딩 윈도우의 왼쪽 시작 위치로 잡기
    leftx_current = np.argmax(histogram[:midpoint-90])
    # 오른쪽 절반 구역에서 흰색 픽셀의 개수가 가장 많은 위치를 슬라이딩 윈도우의 오른쪽 시작 위치로 잡기
    rightx_current = warp_img_w - np.argmax(histogram[histogram.shape[0]:midpoint+87:-1])

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], []

    out_img = np.dstack((lane, lane, lane))*255

    for window in range(nwindows):
        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

        # 슬라이딩 윈도우 박스(녹색박스) 하나 안에 있는 흰색 픽셀의 x 좌표를 모두 모으기
        # 왼쪽과 오른쪽 슬라이딩 박스를 따로 작업하기
        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # 위에서 구한 x좌표 리스트에서 흰색점이 5개 이상인 경우에 한해서 x좌표의 평균값을 구하기
        # 이 값을 위에 쌓을 슬라이딩 윈도우의 중심점으로 사용함(그리고 계속 for 9번 반복)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nz[1][good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)
    
    # 슬라이딩 윈도우의 중심점(x좌표)를 1x/1y, rx/ry에 담아두기(9개를 모두 모으기)
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
    #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)

    # 슬라이딩 윈도우의 중심점(x좌표) 9개를 가지고 2차 함수를 만들어내기
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    # 기존 하얀색 차선 픽셀을 왼쪽과 오른쪽 각각 파란색과 빨간색으로 색상 변경
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)
    
    #return left_fit, right_fit
    return lfit, rfit

def get_pos_from_fit(fit, y=warp_Offset, left=False, right=False):
    a, b, c = fit[0], fit[1], fit[2]

    if a==0 and b==0 and c==0:
        if left:
            pos = 0
        elif right:
            pos = width
    else:
        pos = a*y*y + b*y + c
        
    return pos
    

def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global Width, Height
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    # 이차함수 이용해서 사다리꼴 이미지 외곽선 픽셀 좌표 계산해서
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))
    
    # 사다리꼴 이미지를 칼라(녹색)로 그리고 거꾸로 원근 변환해서 원본 이미지와 오버레이하기
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

# draw_steer
def draw_steer(lane_img, steer_angle):
    global Width, Height
    steer_img = lane_img.copy()
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

    arrow_roi = steer_img[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    steer_img[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    return steer_img

def start():
    global Width, Height, cap, warp_img_w, warp_Offset

    _, frame = cap.read()
    while not frame.size == (Width*Height*3):
        _, frame = cap.read()
        continue

    print("start")

    while cap.isOpened():
        
        _, frame = cap.read()

        image = calibrate_image(frame)
        warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        cv2.imshow(window_title, warp_img)
        left_fit, right_fit = warp_process_image(warp_img)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)
        
        lpos = get_pos_from_fit(left_fit, y=warp_Offset, left=True, right=False)
        rpos = get_pos_from_fit(right_fit, y=warp_Offset, left=False, right=True)
        center = (lpos + rpos) / 2
        angle = warp_img_w/2 - center
        steer_angle = angle * 0.4
        steer_img = draw_steer(lane_img, steer_angle)
        
        #print(lpos, rpos, steer_angle)

        cv2.imshow(window_title, steer_img)

        cv2.waitKey(1)

if __name__ == '__main__':
    start()

