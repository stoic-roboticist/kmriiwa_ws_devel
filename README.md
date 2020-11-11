# kmriiwa_ws_devel

Forked from a repository for master thesis and specialization project in Robotics & Automation, at NTNU Fall 2019, by Charlotte Heggem and Nina Marie Wahl. 

Now further developed during a specialization project in Robotics & Automation at NTNU, fall 2020 by Morten M. Dahl.

![The KMR iiwa](https://img.directindustry.com/images_di/photo-g/17587-12407502.webp)
**Intention:**
This project aims to create a communication API between a KUKA KMR iiwa and ROS2. 
Multiple ROS packages are used for including functionality. 
Navigation2 is used for navigating the mobile vehicle. 
SLAM_Toolbox is used for SLAM.
MoveIt2 is used for path planning for the manipulator. 

A Robotiq gripper is used for picking up objects. 
Four Intel Realsense D435 cameras are mounted to the robot.
If cameras and gripper are to be used, they are launched on a separate onboard computer (Intel NUC). 

**System requirements:** 
- Ubuntu 20.04
- Python 3.8.2
- ROS Foxy


**Required ROS Packages:**
- Gazebo packages
- Navigation2
- MoveIt2
- laserscan_to_pointcloud
- (Ros2 Intel Realsense (ROS2 Wrapper for Intel® RealSense™ Devices))
- (ROS2 Openvino Toolkit (dependent on OpenVino Toolkit))
- (ROS2 Object Analytics)
