#!/usr/bin/env python3

# Copyright 2020 Morten M. Dahl.
# Copyright 2020 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import _thread as thread
import time
import os
import sys
import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from builtin_interfaces.msg import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
import argparse

import laser_geometry.laser_geometry as lg


class KmpLaserScanConverter(Node):
    def __init__(self):
        super().__init__('scan_converter_node')
        self.name='scan_converter_node'

        # Make Publishers for relevant data
        self.pub_pointcloud1 = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.pub_pointcloud2 = self.create_publisher(PointCloud2, 'pointcloud_2', 10)

        # Subscribes to laser scan data
        self.scan1 = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan2 = self.create_subscription(LaserScan, 'scan_2', self.scan_callback, 10)

        # Calls LaserProjection
        lp = lg.LaserProjection()

        self.scan1
        self.scan2



    def scan_callback(self, scan):

        pointcloud = lp.projectLaser(self, scan)

        pub_pointcloud1.publish(pointcloud)

        if scan.header.frame_id() == 'laser_B1_link':
            pub_pointcloud1.publish(pointcloud)
        elif scan.header.frame_id() == 'laser_B4_link':
            pub_pointcloud2.publish(pointcloud)
        else:
            print('Error, scan header invalid.')



def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)

    scan_converter_node = KmpLaserScanConverter()

    rclpy.spin(scan_converter_node)
    try:
        scan_converter_node.destroy_node()
        rclpy.shutdown()
    except:
        print("Error: rclpy shutdown failed")



if __name__ == '__main__':
    main()