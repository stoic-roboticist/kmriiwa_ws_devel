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

        node = rclpy.create_node('tf_listener')

        # Create publisher to publish final pointcloud
        self.publisher_ = self.create_publisher(PointCloud2, 'pc_concatenated', qos_profile = rclpy.qos.qos_profile_sensor_data)

        # Make a LaserProjection object
        self.lp = LaserProjection()

        # Setup for performing PointCloud2 transforms
        print('Initializing TF buffer and listener.')
        self.tf_buffer = tf2_ros.Buffer(None, node = node)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node, spin_thread = False)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        # Listen to TF for 5 seconds
        start_time = time.time()
        print('Listening to /tf and /tf_static for 3 seconds.')
        while (time.time() - start_time < 3.0):
            rclpy.spin_once(node, timeout_sec=0.1)

        try:
            self.transform_B1 = self.tf_buffer.lookup_transform("base_footprint","laser_B1_link", node.get_clock().now())
            print('Got B1 transform.')
            self.transform_B4 = self.tf_buffer.lookup_transform("base_footprint","laser_B4_link", node.get_clock().now())
            print('Got B4 transform.')

        except tf2_ros.LookupException as e:
            print('Error looking up transform:', e, 'Did you remember to launch the joint publisher from kmr_bringup?')
            print('Shutting down.')
            rclpy.shutdown()

        # Subscribes to the two laser scan topics. Might have to change QoS to 10 when subscribing to real laser readings.
        self.subscriber_1 = Subscriber(self, LaserScan, 'scan', qos_profile = rclpy.qos.qos_profile_sensor_data)
        self.subscriber_2 = Subscriber(self, LaserScan, 'scan_2', qos_profile = rclpy.qos.qos_profile_sensor_data)

        # Syncronizes messages from the two laser scanner topics when messages are less than 0.1 seconds apart.
        self.syncronizer = ApproximateTimeSynchronizer([self.subscriber_1, self.subscriber_2], 10, 0.01, allow_headerless=False)

        print('Initialized laser scan syncronizer.')
        
        # Calling callback function when syncronized messages are received.
        self.syncronizer.registerCallback(self.callback)


    def callback(self, scan, scan2):
        # Make two PointCloud2 messages from the scans
        self.pc2_msg1 = self.lp.projectLaser(scan)
        self.pc2_msg2 = self.lp.projectLaser(scan2)

        # Transforms the clouds to the same frame.
        self.pc2_msg1_transformed = PointCloud2()
        self.pc2_msg2_transformed = PointCloud2()

        self.pc2_msg1_transformed = CloudTransform().do_transform_cloud(self.pc2_msg1, self.transform_B1, scan)
        self.pc2_msg2_transformed = CloudTransform().do_transform_cloud(self.pc2_msg2, self.transform_B4, scan)

        # Combine the clouds
        self.pc2_concatenated = LaserProjection().concatenate_clouds(self.pc2_msg1_transformed, self.pc2_msg2_transformed)
        self.pc2_concatenated.header.frame_id = self.transform_B1.header.frame_id

        # Publishes the combined cloud
        self.publisher_.publish(self.pc2_concatenated)

        
        # Publishes the translated pointclouds.
        #self.publisher_.publish(self.pc2_msg1_transformed)
        #time.sleep(0.25)
        #self.publisher_.publish(self.pc2_msg2_transformed)
        #time.sleep(0.25)


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



###################################################################################################################################################################################

# Class for creating PointCloud2 messages from LaserScan messages

class LaserProjection():

    LASER_SCAN_INVALID   = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE      = 0x00 # Enable no channels
        INTENSITY = 0x01 # Enable "intensities" channel
        INDEX     = 0x02 # Enable "index"       channel
        DISTANCE  = 0x04 # Enable "distances"   channel
        TIMESTAMP = 0x08 # Enable "stamps"      channel
        VIEWPOINT = 0x10 # Enable "viewpoint"   channel
        DEFAULT   = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0
        self.__cos_sin_map = np.array([[]])

        self._DATATYPES = {}
        self._DATATYPES[PointField.INT8]    = ('b', 1)
        self._DATATYPES[PointField.UINT8]   = ('B', 1)
        self._DATATYPES[PointField.INT16]   = ('h', 2)
        self._DATATYPES[PointField.UINT16]  = ('H', 2)
        self._DATATYPES[PointField.INT32]   = ('i', 4)
        self._DATATYPES[PointField.UINT32]  = ('I', 4)
        self._DATATYPES[PointField.FLOAT32] = ('f', 4)
        self._DATATYPES[PointField.FLOAT64] = ('d', 8)

    def projectLaser(self, scan_in,
            range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.
        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.
        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)


    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
            self.__angle_max != scan_in.angle_max):

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max
            
            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance =  idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
            len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

        cloud_out = self.create_cloud(scan_in.header, fields, points)

        return cloud_out


    def concatenate_clouds(self, cloud1, cloud2):
        '''
        This requires 2 clouds in the same frame.
        '''
        points_1 = self.read_points(cloud1)
        points_2 = self.read_points(cloud2)

        points_concatenated = []

        for point in points_1:
            points_concatenated.append(point)

        for point in points_2:
            points_concatenated.append(point)

        header = cloud1.header

        fields = cloud1.fields

        concatenated_cloud = self.create_cloud(cloud1.header, fields, points_concatenated)
        return concatenated_cloud


    def create_cloud(self, header, fields, points):
        """
        Create a L{sensor_msgs.msg.PointCloud2} message.
        @param header: The point cloud header.
        @type  header: L{std_msgs.msg.Header}
        @param fields: The point cloud fields.
        @type  fields: iterable of L{sensor_msgs.msg.PointField}
        @param points: The point cloud points.
        @type  points: list of iterables, i.e. one iterable for each point, with the
                       elements of each iterable being the values of the fields for 
                       that point (in the same order as the fields parameter)
        @return: The point cloud.
        @rtype:  L{sensor_msgs.msg.PointCloud2}
        """

        cloud_struct = struct.Struct(self._get_struct_fmt(False, fields))

        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0
        for p in points:
            pack_into(buff, offset, *p)
            offset += point_step

        return PointCloud2(header=header,
                           height=1,
                           width=len(points),
                           is_dense=False,
                           is_bigendian=False,
                           fields=fields,
                           point_step=cloud_struct.size,
                           row_step=cloud_struct.size * len(points),
                           data=buff.raw)

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in self._DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
            else:
                datatype_fmt, datatype_length = self._DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """
        Read points from a L{sensor_msgs.PointCloud2} message.
        @param cloud: The point cloud to read from.
        @type  cloud: L{sensor_msgs.PointCloud2}
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        #assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                        if not has_nan:
                            yield p
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                            if not has_nan:
                                yield p
                        offset += point_step
        else:   
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        yield unpack_from(data, offset)
                        offset += point_step


class CloudTransform():
    def transform_to_quat_vec(self, t):
        q = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        v = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        qv = [q,v]
        return qv

    # PointStamped
    def do_transform_cloud(self, cloud, transform, original_scan):
        qv = self.transform_to_quat_vec(transform)

        points_out = []

        q = qv[0]
        v = qv[1]

        rot_z = np.array([[-1,0,0, 0],
                          [0,-1,0, 0],
                          [0,0,1, 0],
                          [0, 0, 0, 1]])

        rot_y = np.array([[-1,0,0,0],
                          [0,1,0,0],
                          [0,0,-1,0],
                          [0,0,0,1]])

        refl_x = np.array([[-1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        refl_y = np.array([[1, 0, 0, 0],
                           [0, -1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        rot = rot_z @ rot_y

        T = np.array([[q[1]**2 + q[2]**2 - q[3]**2 - q[0]**2, 2*(q[2]*q[3] - q[1]*q[0]), 2*(q[2]*q[0] + q[1]*q[3]),                                       -v[0]],
                      [2*(q[2]*q[3] + q[1]*q[0]),                                        q[1]**2 - q[2]**2 + q[3]**2 - q[0]**2, 2*(q[3]*q[0] - q[1]*q[2]),v[1]],
                      [2*(q[2]*q[0] - q[1]*q[3]),                                        2*(q[3]*q[0] + q[1]*q[2]), q[1]**2 - q[2]**2 - q[3]**2 + q[0]**2,v[2]],
                      [0,                                                                0,                      0,                                       1.0]])
        print(T)

        # Had to multiply with a reflection matrix about the x-axis.
        for p_in in LaserProjection().read_points(cloud):
            p_out = np.array((T @ [p_in[0], p_in[1], p_in[2], 1.0])) @ refl_x
            p_out = [p_out[0], p_out[1], p_out[2], 0.0, p_in[4]]
            points_out.append(p_out)

        res = LaserProjection().create_cloud(original_scan.header, cloud.fields, points_out)
        return res

###################################################################################################################################################################################


if __name__ == '__main__':
    main()