#!/usr/bin/env python3

import _thread as thread
import time
import sys
import math
import rclpy
from rclpy.node import Node
from kuka_communication.msg import KmpStatusdata
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from scripts.TCPSocket import TCPSocket
from scripts.UDPSocket import UDPSocket

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpStatusNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_odometry_node')

        if robot == 'KMR1':
            port = 30001
            ip = 1010
        elif robot == 'KMR2':
            port = 1223
            ip= 1212
        else:
            port=None
            ip=None


        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port)
        elif connection_type == 'UDP':
            self.soc=UDPSocket(ip,port)
        else:
            self.soc=None

        self.last_odom_timestamp = 0

        # Make Publisher for statusdata
        self.pub_kmp_statusdata = self.create_publisher(KmpStatusdata, 'kmp_statusdata', qos_profile_sensor_data)


        while not self.soc.isconnected:
            pass
        print('Ready to start')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            self.status_callback(self.pub_kmp_statusdata, self.soc.kmp_statusdata)



    def status_callback(self,publisher,values):
        msg = KmpStatusdata()
        msg.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)
        msg.header.frame_id = "baselink"


        #TODO: Fyll inn med riktig statusdata - dette maa ogsaa gjores i selve meldingsfila - den er naa random.
        msg.data= str(values[0])

        publisher.publish(msg)


    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    print(argv)
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_statusdata_node = KmpStatusNode(args.connection,args.robot)

    rclpy.spin(kmp_statusdata_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        kmp_statusdata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
