ros-indigo-qbo-packages
=======================

Qbo_stack moved to indigo and modified for more standard use.

###############
# Disclaimer

 This code is given as-is, no reponsability will be taken in case of bug/damage caused to you robot.
 I am working on a standard Qbo Pro Evo robot, adaptation may be needed if your robot is different.

##############

Installation:
=============

- Install Ubuntu 14.04 (I made a bootable USB stick using http://unetbootin.sourceforge.net/ Don't forget to enable booting from USB in the boot options (for me, F2 when starting, enable USB, then F10 and boot from card reader))
- Install ROS Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu)
- Install a few dependencies 
sudo apt-get install libpam-systemd libsystemd-daemon0 libsystemd-login0 libudev1 systemd-services udev ros-indigo-uvc-camera ros-indigo-camera-calibration-parsers ros-indigo-image-view
- add your user in dialout and video groups to be able to use USB and video ports. (My user name is qbobot, adapt to your setup)
sudo usermod -a -G dialout qbobot
sudo usermod -a -G video qbobot
(to list the groups you are in : cat /etc/group | grep qbobot )
- Create a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace). I called the workspace ros instead of catkin_ws, but this has no importance.
In the src folder, there is a CMakeLists.txt link. Save it somewhere else, then remove src folder.
- In your catkin workspace, clone this directory in a folder named src:
  git clone git@github.com:HumaRobotics/ros-indigo-qbo-packages.git src
Inside the src folder, put back the CMakeLists.txt you saved before.
- compile:
  catkin_make
- run (don't forget to do a 'source ./devel/setup.bash' in your catkin_workspace in each new terminal). 
Terminal 1 : roscore
Terminal 2 : roslaunch qbo_arduqbo qbo_arduqbo_default.launch 
You can now look at published topics and move the motors and light up the mouth and nose


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
- floor_controller was empty and unused, it has been deleted
- Floor sensors gives a distance in meters and not centimeters
- SRF sent data everytime they receive it, even if it is 0 (no obstacle seen). This behavior can be modified through publish_if_obstacle parameter
- IMU has a parameter last_calibrated containing a time (or -1 if calibrating failed). The calibrated publisher has been removed.
- IMU values computation has been modified (some errors in previous version ?)
- NodeHandle nh has been added to controller_class
- LCD message replaced by string message
- LCD can be commanded by '1hello' to write on 2nd line, '2hello' to write hello on 3rd line... Or by putting all 4 lines at once, separated by /. Example: hello/how are you//fine
- mics_controller use UInt16MultiArray instead of qbo_arduqbo/NoiseLevel.msg
- mouth is now controlled by a ByteMultiArray of 4 bytes (one for each led line)
command line example: rostopic pub -1 /cmd_mouth std_msgs/ByteMultiArray "{}" [0b10001,0b10001,0b01010,0b00100]
- nose controller takes a UInt16 standard message. Values above 3 are unused (0 : off, 1 : red, 2:green, 3: yellow)

Difference between previous qbo_face_tracking and new qbo_face_detection node:
- Nose does not light up. This node is completely independant from arduqbo
- Use geometry_msgs/PoseStamped instead of FacePosAndDist message
- in message, x and y are values in the camera image, between -1 and 1
- A distance of -1 means no face has been detected. Distance measurement may be bad, because of poblems with the camera projection matrix

New qbo_move package:
This package is divided in 3 nodes:
- qbo_move_head to control the head servos (neck and eyelids). A special tracker topic allows you to follow an objet seen from the camera with the head (see qbo_track_face_head.launch)
- qbo_move_base to control the base. TODO : a special mode allows to allign with the head, useful when tracking)
- qbo_avoid_obstacles. TODO : can be used between qbo_move_base and qbo_arduqbo to prevent moves going to obstacles
