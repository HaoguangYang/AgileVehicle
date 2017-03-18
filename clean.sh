#!/bin/bash
echo "------------ CLEANING STEERING WHEEL DRIVERS ------------"
cd ./LogitechFFDrivers
make clean

echo "----------------- CLEANING ROS PACKAGES -----------------"
cd ../ROS
catkin_make clean

echo "--------------------- DONE CLEANING ---------------------"

