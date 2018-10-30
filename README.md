# Maxon EPOS2 Controller

This ROS node can be used to communicate with the EPOS2 Controller from Maxon. EposCommunication wrapper is independent of ROS and more functionality from EPOS2 Command Library can be added in the same way.

In the current version, this software only activates Maxon's ProfilePositionMode to control position. 

This is a catkin workspace ROS node, which allows absolute position control for Maxon EPOS2 with DCX motors. Tested on Ubuntu 16.04 and ROS Kinetic Kame.

## Install Maxon EPOS Linux Library Driver from Maxon
Installation guide: https://www.maxonmotor.ch/medias/sys_master/root/8828279488542/EPOS-Command-Library-En.pdf

Download Library from: https://www.maxonmotor.ch/maxon/view/product/control/Positionierung/530239

## Start node
`cd catkin_ws`

`roslaunch maxon_epos2 maxon_epos2.launch`

## Commands
Homing (necessary before it reacts to position commands):

`rosservice call /maxon/epos_homing_service`

Set position:

`rosservice call /maxon/epos_control_service "position_setpoint: 4" (position in mm between -47mm and 0 mm extension)`

Get position and velocity:

`rostopic echo /maxon/epos_info_topic"`

## Important note
- Set USB Port in EposCommunication::SetDefaultParameters() if necessary.
Find devices with:

`dmesg | grep tty`

- Replace "xxx" in .launch file by EPOS2 serial number.


