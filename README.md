# 23rd Korea AAM Tech Challenge - Konkuk Univ. Team ASEC 
2025 제23회 한국로봇항공기대회 건국대학교 ASEC팀 전체 소프트웨어 코드

## rohang25_test package
**대회 최종 코드**
- PX4 Mission mode -> Offboard mode switching
- Object tracking pipeline using YOLOv11 detection data (NED transformed)
- Target rescue sequence (by timer, ~~sensors~~)
- VTOL Quad-mode smooth velocity guidance
- Precision landing sequence
- Prints current status: current pos / vel / heading / mission status / object status / etc.

## rohang25 package
**Original rohang25 offboard guidance code**
- Auto arm/takeoff
- Full offboard guidance throughout the whole mission
- _Mission sequence not included_ (object tracking, rescue, precision landing)
- Auto-land

## yolov11 & yolov11_msgs 
**Object detection package**
- Detects all targets (Target, release marker, vertiport marker)
- Sends detection data through custom ROS2 msg (yolov11_msgs/Detection.msg)

## geolocation_kf_25 package
**Frame conversion package**
- Receives YOLOv11 detection data (pixel coordinates)
- Conversion from pixel -> NED local frame 
- Kalman filtering
- Sends filtered data to main package (rohang25_test)


## gimbal_control_25 package
**Gimbal control node**
- _Package developed but not used_
- Receives object detection data
- Sends command to gimbal to track object (Pan & Tilt)

## Submodules
**gscam**
- Enables gimbal communication

**px4_msgs**
- basic PX4 messages to ROS2

