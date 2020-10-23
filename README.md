# kmriiwa_ws_devel

Forked from a repository for master thesis and specialization project in Robotics & Automation, at NTNU Fall 2019, by Charlotte Heggem and Nina Marie Wahl. 

Now further developed during a specialization project in Robotics & Automation at NTNU, fall 2020 by Morten M. Dahl.

**Intention:**
This project aims to create a communication API between a KUKA robot, KMR iiwa, and ROS2. 
Multiple ROS packages are used for including functionality. 
Navigation2 is used for navigating the mobile vehicle. 
Cartographer and RTAB-Map is used for SLAM. 
MoveIt2 is used for path planning for the manipulator. 


Multiple Intel Realsens D435 cameras are used to provide better moving and detection of objects. 
A Robotiq gripper is used for picking up objects. 
The cameras and gripper are launched at a separate onboard computer (Intel NUC). 

**System requirements:** 

- Ubuntu 20.04
- Python 3.8.2
- ROS Foxy


**Required ROS Packages:**
- Gazebo packages
- Navigation2
    - SMAC_planner (?)
- MoveIt2
- Cartographer
- RTAB-Map ROS wrapper (dependent on RTAB-Map)
- Ros2 Intel Realsense (ROS2 Wrapper for Intel® RealSense™ Devices)
- ROS2 Openvino Toolkit (dependent on OpenVino Toolkit)
- ROS2 Object Analytics