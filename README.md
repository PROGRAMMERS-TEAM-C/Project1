# 시뮬레이터 Code
## Video
---
1. 맵 주행
2. 차선 인식
3. 주차 미션
(https://youtu.be/Qs8GvizAjug)https://www.youtube.com/watch?v=Qs8GvizAjug
[![주차 미션](http://img.youtube.com/vi/Qs8GvizAjug/0.jpg)](https://www.youtube.com/watch?v=Qs8GvizAjug) 
## Environment & Installation
---
- Ubuntu 18.04
- ROS Melodic
- ar-track-alvar v0.7.1
~~~bash
$ sudo apt update
$ sudo apt-get install ros-melodic-ar-track-alvar
~~~
- pygame v1.9.6
~~~bash
$ sudo apt update
$ sudo apt-get install python-pip
$ pip2 install pygame==1.9.6
~~~
- pillow v6.2.2 
~~~bash
$ pip2 install pillow==6.2.2
~~~

## Structure
---
~~~
src
  └─ ar_viewer
  │    └─ launch
  │    │    └─ ar_drive.launch
  │    │    └─ ar_parking_try1.launch
  │    │    └─ ar_parking_try2.launch
  │    │    └─ ar_parking_tryout.launch
  │    └─ src
  │    │    └─ ar_drive.py
  │    │    └─ ar_parking_try1.py
  │    │    └─ ar_parking_try2.py
  │    │    └─ ar_parking_tryout.py
  └─ line_drive
  │    └─ src
  │    │    └─ kmu_track.mkv
  │    │    └─ line_drive.py
  │    │    └─ steer_arrow.png
  └─ xycar_sim_drive
  │    └─ launch
  │    │    └─ xycar_sim_drive_try.launch
  │    │    └─ xycar_sim_drive_tryout.launch
  │    │    └─ xycar_sim_drive_tryout_obstacle.launch
  │    └─ maps
  │    │    └─ ㄹ.so
  │    │    └─ ㄹ_obstacle.so
  │    │    └─ ㅁ.so
  │    │    └─ 凹.so
  │    └─ src
  │    │    └─ main.py
  │    │    └─ ultra_driver_try.py
  │    │    └─ ultra_driver_tryout.py
  │    │    └─ ultra_driver_obstacle.py
  └─ xycar_sim_parking
       └─ src
            └─ main.py
~~~

## Usage
---
### 1. 맵 주행
~~~bash
$ roslaunch xycar_sim_drive_try.launch
~~~
### 2. 차선 인식
~~~bash
$ cd src/line_drive/src
$ python line_drive.py
~~~
### 3. 주차 미션
~~~bash
$ roslaunch ar_parking_try1.launch
~~~

## Goal
---
### 1. 뱀 모양 맵을 벽에 부딪히지 않고 총 3바퀴 주행
### 2. 차선을 벗어나지 않고 주행
### 3. ar태그를 인식하여 주차 공간에 알맞게 주차

## Procedure
---
### 1. 맵 주행
- 
- 
- 
### 2. 차선 인식
- 
- 
### 3. 주차 미션

## Limitations
---
### 1. 맵 주행
### 2. 차선 인식
### 3. 주차 미션

## What I've learned
---
### 1. 맵 주행
### 2. 차선 인식
### 3. 주차 미션
