#!/bin/bash

cd ~
sleep 3
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/script/
#bash ./ubuntu_16.04_base.sh
echo 安装 ROS 基本环境
sleep 3
bash ./ros_indigo_base.sh
echo 安装 ROS 扩展环境
sleep 3
bash ./ros_indigo_ext.sh

echo 安装 HandsFree ROS 
sleep 3
source /opt/ros/indigo/setup.bash
cd ~/handsfree/handsfree_indigo/src/
catkin_init_workspace
sleep 3
cd ~/handsfree/handsfree_indigo
catkin_make
echo "source ~/handsfree/handsfree_indigo/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/handsfree

