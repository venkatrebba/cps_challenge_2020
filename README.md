# Accessing the OpenUAV simulation testbed 
Register on https://cps-vo.org/group/CPSchallenge to get access to pre-configured docker container with 3D graphics, ROS, Gazebo, and PX4 flight stack. 

# Scripts to control drone in PX4 Gazebo environment

Look at `docking_demo.py` for code to attach and detach an object with a PX4 drone.
This script will also set mode to Offboard and arm the vehicle.

This script publishes to attach and detach commands, which require the following repository code to be setup and running.
https://github.com/Open-UAV/gazebo_ros_link_attacher 



Run `multi_vehicle_position_control_demo.py` in catkin_ws/src/cps_challenge_2020/scripts folder to move the drone through a series of position setpoints illustrating the key tasks for Phase II mission scenario. The rover can also be switched to offboard and armed for it to travel north with an average speed of 1 m/s. 
This script will need you manually set the mode to offboard and arm the vehicle.
Commands to set mode and arm the vehicle.
```
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /mavros/cmd/arming "value: true"
```
 	
  

# RTAB Map:

Mapped rock with RTAB Map.


Screenshot
![Screen Shot 2022-04-14 at 6 52 04 PM](https://user-images.githubusercontent.com/32699857/163505339-5dee14ba-621e-488d-b939-f570112dea93.png)

Full Video:
