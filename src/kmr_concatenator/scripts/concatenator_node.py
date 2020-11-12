#!/usr/bin/env python3

# Copyright 2020 Morten Melby Dahl.
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

import sys
import argparse
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import time

# Import scripts
from cloud_transform import *
from laser_to_pointcloud import *

# rclpy and math
import rclpy
import numpy as np
import math
from rclpy.node import Node

# For transforming LaserScan to PointCloud2
import struct
import ctypes

# Messages
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import std_msgs

# Packages to transform pointcloud
import tf2_ros
import launch_ros.actions

# Message syncronization
from message_filters import Subscriber, ApproximateTimeSynchronizer


class LaserConcatenator(Node):

    def __init__(self):
        super().__init__('laser_concatenator')
        self.name = 'laser_concatenator'

        # Create publisher to publish final pointcloud
        self.publisher_ = self.create_publisher(PointCloud2, 'pc_concatenated', qos_profile = rclpy.qos.qos_profile_sensor_data)

       
        # Setup for listening to transformation messages over /tf and /tf_static.
        print('Initializing TF buffer and listener.')
        node = rclpy.create_node('tf_listener')
        self.tf_buffer = tf2_ros.Buffer(None, node = node)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node, spin_thread = False)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        # Listen to TF for 5 seconds
        start_time = time.time()
        print('Listening to /tf and /tf_static for 3 seconds.')
        while (time.time() - start_time < 3.0):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Gets the transform message from original to goal frame.
        try:
            self.transform_B1 = self.tf_buffer.lookup_transform("base_footprint","laser_B1_link", node.get_clock().now())
            print('Got B1 transform from', self.transform_B1.child_frame_id, 'to', self.transform_B1.header.frame_id)
            self.transform_B4 = self.tf_buffer.lookup_transform("base_footprint","laser_B4_link", node.get_clock().now())
            print('Got B4 transform from', self.transform_B4.child_frame_id, 'to', self.transform_B4.header.frame_id)

        except tf2_ros.LookupException as e:
            print('Error looking up transform:', e, 'Did you remember to launch the publisher from kmr_bringup?')
            print('Shutting down.')
            rclpy.shutdown()

        if (self.transform_B1.header.frame_id != self.transform_B4.header.frame_id):
        	print("Warning: B1 and B4 transformations does not have the same goal frame. Concatenation not possible.")
        	rclpy.shutdown()

        # Generate transformation matrices in the form of NumPy arrays from geometry_msgs.msg.TransformStamped
        self.T1 = CloudTransform().generate_transform(self.transform_B1)
        self.T4 = CloudTransform().generate_transform(self.transform_B4)

        # Subscribes to the two laser scan topics. Might have to change QoS to 10 when subscribing to real laser readings.
        # if you are using Gazebo, final arguemtn should be qos_profile = rclpy.qos.qos_profile_sensor_data.
        self.subscriber_1 = Subscriber(self, LaserScan, 'scan', qos_profile = rclpy.qos.qos_profile_sensor_data)
        self.subscriber_2 = Subscriber(self, LaserScan, 'scan_2', qos_profile = rclpy.qos.qos_profile_sensor_data)

        # Syncronizes messages from the two laser scanner topics when messages are less than 0.01 seconds apart.
        # Adjust the time to a lower setting to increase the accuracy of the map at the cost of output frequency.
        self.synchronizer = ApproximateTimeSynchronizer([self.subscriber_1, self.subscriber_2], 10, 0.01, allow_headerless=False)

        print('Initialized laser scan syncronizer.')
        
        # Calling callback function when syncronized messages are received.
        self.synchronizer.registerCallback(self.callback)


    def callback(self, scan, scan2):
        # Make two PointCloud2 messages from the scans
        self.pc2_msg1 = LaserToPointcloud().projectLaser(scan)
        self.pc2_msg2 = LaserToPointcloud().projectLaser(scan2)

        # Transforms the clouds to the same frame.
        self.pc2_msg1_transformed = CloudTransform().do_transform_cloud(self.pc2_msg1, self.T1, scan, True)
        self.pc2_msg2_transformed = CloudTransform().do_transform_cloud(self.pc2_msg2, self.T4, scan, True)

        # Combine the clouds
        self.pc2_concatenated = LaserToPointcloud().concatenate_clouds(self.pc2_msg1_transformed, self.pc2_msg2_transformed)
        self.pc2_concatenated.header.frame_id = self.transform_B1.header.frame_id

        # Publishes the combined cloud
        self.publisher_.publish(self.pc2_concatenated)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    args = parser.parse_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)
    concatenator_node = LaserConcatenator()

    rclpy.spin(concatenator_node)

    try:
        concatenator_node.destroy_node()
        rclpy.shutdown()
    except:
        print("Error: rclpy shutdown failed")


if __name__ == '__main__':
    main()