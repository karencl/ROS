
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.8
python3.8 --version
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1
python3
python3 -m pip install pip
pip3 --version
sudo apt-get update
sudo apt-get install gcc python3.8-dev
pip3 install yolov5
pip3 install rospkg
pip3 install rospy
sudo apt install ros-melodic-vision-msgs
sudo apt-get rps=melodic-rosbash
sudo apt-get install ros=melodic-rosbash
cd catkin_ws
catkin_make
find_package(catkin REQUIRED COMPONENTS(OpenCV)
pip3 install onxx>=1.12.0
