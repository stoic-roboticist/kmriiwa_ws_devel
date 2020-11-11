# kmriiwa_ws_devel
<img align="right" src="https://img.directindustry.com/images_di/photo-g/17587-12407502.webp" height="250"/>
Forked from a repository for master thesis and specialization project in Robotics & Automation, at NTNU Fall 2019, by Charlotte Heggem and Nina Marie Wahl. 

Now further developed during a specialization project in Robotics & Automation at NTNU, fall 2020 by Morten M. Dahl.

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

## Guide
All the packages inside the src folder are made to enable the use of a KMR iiwa robot with ROS2. A simple description of all the packages and a quick-start guide follows.
### kmr_behaviortree
The package is made to enable the use of behaviortrees with the KMR. ![Behaviortree.CPP](https://www.behaviortree.dev/) interfaces with Navigation2 and MoveIt2, which makes it simple to make pre-installed programs.

### kmr_bringup
This package publises the transformation between different parts of the robot. This is required when odometry and laser scan data needs to be expressed in different frames of the robot.

### kmr_communication
To establish communication between the KMR and ROS2, this package sets up a TCP or UDP connection which receives and sends information between the two systems.

### kmr_concatenator
As the preferred SLAM software SLAM_Toolbox only takes in data from a single LaserScan topic, data from the two laser scanners on the KMR has to be concatenated and transformed. This package concatenates the data by converting between LaserScan and PointCloud2 and transforming the individual points before adding them back together. It is also supported by a node from the 'laserscan_to_pointcloud' package with the pointcloud_to_laserscan_node.

### kmr_manipulator
To enable the use of a Robotiq gripper outside of the KUKA system, this package was developed. It takes in ROS commands and translates them into movement of the gripper.

### kmr_moveit2
This package enables trajectory generation for the iiwa arm by integrating MoveIt2.

### kmr_msgs
Custom messages used for actions and moving the gripper.

### kmr_navigation2
Enables the use of Navigation2 with the KMR.

### kmr_simulation
Simulate the KMR iiwa in Gazebo.

### kmr_slam
Enables both RTAB-Map and Cartographer to be used with the robot. This is not the preferred SLAM method, and better results have been achieved by using SLAM_Toolbox.

### kmr_slam_toolbox
Integrates the use of SLAM_Toolbox with the robot. A requirement to run this package is to have kmr_concatenator running in order to get input from both lasers. Further instructions inside the package.

### kmr_sunrise
Code which is installed on the KMR through KUKA Workbench.

### kmr_sunrise_original
The original code from which the updated kmr_sunrise code was based.

### Quick-start guide
Currently, only the packages that enable mapping of an environment using ![SLAM_Toolbox](https://github.com/SteveMacenski/slam_toolbox/) has been enabled for use in ROS2 'Foxy Fitzroy'. Other packages, such as Navigation2 and MoveIt2, are yet to be fully tested with the current version.

To launch the robot in a simulated environment, simply run
```
ros2 launch kmr_simulation gazebo.launch.py
```
You should now see the robot visualized in Gazebo. 

If you wish to use a real KMR, run
```
ros2 launch kmr_communications sunrise_communication.launch.py
ros2 launch kmr_concatenator concatenator.launch.py
```
Then run the KMRiiwaSunriseApplication.java application on the KMR. 
It is a requirement that you have both the computer running ROS2 and the KMR connected to the same access point and have the correct IP address of your computer configured in both the kmr_communications package and the java application on the KMR.

To move the robot using the keyboard, run
```
ros2 run kmr_navigation2 twist_keyboard.py
```

Regardless of whether you simulate or are running the real KMR, you should now see topics such as /scan and /scan_2 running if you type `ros2 topic list`.

To get the transformations between different joints of the robot, run
```
ros2 launch kmr_bringup state_publisher.launch.py
```
This is required in order to get kmr_concatentator to work. kmr_concatenator takes input from the two laser scanners on the robot and concatenates them into a single LaserScan message expressed in a single frame.

To concatenate the laser scan messages, run
```
ros2 launch kmr_concatenator concatenator.launch.py
```
By again running `ros2 topic list`, you should now see /pc_concatenated and /scan_concatenated. /pc_concatenated is created by the concatenator_node, and /scan_concatenated is created by the pointcloud_to_laserscan_node using the concatenated point clouds. 

To create maps using SLAM of the simulated KMR, run
```
ros2 launch kmr_slam_toolbox gazebo_online_async_launch.launch.py
```
If you are using the real KMR, run
```
ros2 launch kmr_slam_toolbox KMR_online_async_launch.launch.py
```

To visualize the map you are creating, we need to launch rviz. This can be done by running
```
ros2 launch kmr_bringup rviz.launch.py
```
You should now be able to see the map appear as you move the robot.

## Examples
By running communications and SLAM_Toolbox, I was able to map out parts of the MANULAB at NTNU in Trondheim, Norway.
<p align="center">
  <img src="https://raw.githubusercontent.com/MortenMDahl/kmriiwa_ws_devel/foxy/MANULAB.PNG" height="2000"/>
</p>
