# Autonomous driving unit with Puzzlebot and YOLO

## Authors

Esteban Padilla Cerdio
Karen Cebreros López
Andrea González Arredondo
Naomi Estefanía Nieto Vega
Aranza Leal Aguirre

## Execution

 1) Install YOLOv5 from Ultralitics
 2) Download <a href="https://manchesterrobotics-my.sharepoint.com/:f:/g/personal/mario_mtz_manchester-robotics_com/EqsMKMm4UqJCmhnUHEI-xE0B6J-UlYj9kd1KGNxGt3T5AQ?e=kehrqy">Manchester Robotics LTD's YOLOv5 Library</a> and place it in Home
 3) Install the Requirements for YOLOv5, found in Requirements.txt inside Manchester's library
 4) Install OpenCV, Numpy and Itertools
 5) Place this folder in an Ubuntu-ROS machine, inside catkin_ws/src
 6) Go to catkin_ws
 7) Execute ```catkin_make```
 8) Execute YOLO node with ```roslaunch final_challenge yolo.launch```
 9) Execute main program with ```roslaunch final_challenge main.launch```