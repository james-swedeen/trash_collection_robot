FROM osrf/ros:noetic-desktop-full
ARG USER=user
WORKDIR /home/${USER}/ros_ws
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update
RUN apt-get install -y ros-noetic-moveit ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-roboticsgroup-upatras-gazebo-plugins ros-noetic-actionlib-tools terminator python3-pip
RUN pip install flask flask-ask-sdk ask-sdk

RUN apt-get install -y ros-noetic-desktop-full
RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt-get install -y ros-noetic-dynamixel-sdk ros-noetic-pointcloud-to-laserscan
RUN apt-get install -y ros-noetic-joy ros-noetic-moveit-visual-tools
RUN apt-get install -y ros-noetic-turtlebot3
RUN apt-get install -y ros-noetic-realsense2-camera

RUN rosdep update

