ros-indigo-qbo-packages
=======================

Qbo_stack moved to indigo and modified for more standard use

###############
# Disclaimer

 This code is given as-is, no reponsability will be taken in case of bug/damage caused to you robot
 I am working on a standard Qbo Pro Evo robot, adaptation may be needed if your robot is different

##############

Installation:
=============

- Install Ubuntu14 (TODO link)
- Install ROS Indigo (TODO link)
- Install a few dependencies (TODO)
- Create a catkin workspace (TODO)
- clone this directory in a folder named src:
  git clone git@github.com:HumaRobotics/ros-indigo-qbo-packages.git src
- compile:
  catkin_make
  you may have to compile first all messages and services before compiling executables (TO IMPROVE)
- run (TODO)


Contents:
=========

#qbo_arduqbo

This node communicates with the Q.Boards, so can move the base and the head, 
light up the nose and the mouth, write on the LCD and read sensor values

launch with:
roslaunch qbo_arduqbo qbo_arduqbo_default.launch

Differences with previous qbo_arduqbo node:
- cereal_port has been integrated inside qbo_arduqbo
- Battery_status message is not used anymore. Battery level is published in /qbo_arduqbo/battery_level as a std_msgs/Float32 and robot status is published in /qbo_arduqbo/status as a std_msgs/Int8
- controller names have been changed in the parameters
- qboduino driver has been cleaned and commented in english
- floor_controller was empty an dunused, it has been deleted
- Floor sensors gives a distance in meters and not centimeters
