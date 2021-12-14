#!/bin/bash

cd $HOME/open_manipulator_ws/src
#git  clone https://github.com/GerardHarkemaAvans/open_manipulator.git
sudo apt update
sudo apt-get install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench*
sudo apt-get install ros-melodic-robotis-manipulator
cd $HOME/open_manipulator_ws/
catkin b

