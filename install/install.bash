#!/bin/bash

cd $HOME/open_manipulator_ws/src
git  clone https://github.com/GerardHarkemaAvans/open_manipulator.git

cd $HOME/open_manipulator_ws/
catkin b

