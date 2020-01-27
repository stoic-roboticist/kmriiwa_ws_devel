#!/usr/bin/env python3
import _thread as thread
import threading
import time
import os
import rclpy
from rclpy.node import Node
import socket




def cl_black(msge): return '\033[30m' + msge + '\033[0m'
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_orange(msge): return '\033[33m' + msge + '\033[0m'
def cl_blue(msge): return '\033[34m' + msge + '\033[0m'
def cl_purple(msge): return '\033[35m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'
def cl_lightgrey(msge): return '\033[37m' + msge + '\033[0m'
def cl_darkgrey(msge): return '\033[90m' + msge + '\033[0m'
def cl_lightred(msge): return '\033[91m' + msge + '\033[0m'
def cl_lightgreen(msge): return '\033[92m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_lightblue(msge): return '\033[94m' + msge + '\033[0m'
def cl_pink(msge): return '\033[95m' + msge + '\033[0m'
def cl_lightcyan(msge): return '\033[96m' + msge + '\033[0m'


#######################################################################################################################
#   Class: Kuka iiwa TCP communication    #####################
class UDPSocket:
    #   M: __init__ ===========================
    def __init__(self,ip,port):
        self.BUFFER_SIZE = 4096
        #self.BUFFER_SIZE = 10000
        self.isconnected = False
        self.isFinished = (False, None)
        self.hasError = (False, None)
        self.isready = False
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.udp = None

        #TODO: Do something with isready, which is relevant for us.
        threading.Thread(target=self.connect_to_socket).start()
        #try:
        #    threading.Thread(target=self.connect_to_socket).start()
        #except:
        #    print(cl_pink("Error: ") + "Unable to start connection thread")

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):
        # TODO: REPLACE THIS WHEN CONFIG.TXT IS FIXED
        ros_host="192.168.10.116"
        ros_port = 30001

        os.system('clear')
        print(cl_pink('\n=========================================='))
        print(cl_pink('<   <  < << INITIALIZE UDPconnection>> >  >   >'))
        print(cl_pink('=========================================='))
        print(cl_pink(' KUKA API for ROS2'))
        print(cl_pink('==========================================\n'))

        print(cl_cyan('Starting up on:'), 'IP:', ros_host, 'Port:', ros_port)
        try:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(0.1)
            self.udp.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
            self.udp.bind((ros_host, ros_port))
        except:
            print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", ros_host, ros_port)
            os._exit(-1)


        print(cl_cyan('Waiting for a connection...'))
        while (not self.isconnected):
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                self.isconnected = True
            except:
                t=0
        print(cl_cyan('Connection from: '), self.client_address)
        print(cl_cyan('Message: '), data.decode('utf-8'))

        self.udp.sendto("hello KUKA".encode('utf-8'), self.client_address)
        print("Responded KUKA")


        timee = time.time() #For debugging purposes
        count = 0
        while self.isconnected:
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                data = data.decode('utf-8')
                last_read_time = time.time()  # Keep received time
                # Process the received data package
                cmd_splt=data.split(">")[1].split()
                if len(cmd_splt) and cmd_splt[0] == 'isFinished':
                    if cmd_splt[1] == "false":
                        self.isFinished = False
                    elif cmd_splt[1] == "true":
                        self.isFinished = True
                if len(cmd_splt) and cmd_splt[0] == 'hasError':
                    if cmd_splt[1] == "false":
                        self.hasError = False
                    elif cmd_splt[1] == "true":
                        self.hasError = True
                if len(cmd_splt) and cmd_splt[0] == 'odometry':
                        self.odometry = cmd_splt
                if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                    if cmd_splt[2] == '1801':
                        self.laserScanB1.append(cmd_splt)
                        # print(cmd_splt)
                        count = count + 1
                        print(count)
                    elif cmd_splt[2] == '1802':
                        self.laserScanB4.append(cmd_splt)
                        count = count + 1
                        print(count)

            except:
                t=0
                #elapsed_time = time.time() - last_read_time
                #if elapsed_time > 5.0:  # Didn't receive a pack in 5s
                #  print("exception!!")
                #  self.isconnected = False
                #  print(cl_lightred('No packet received from iiwa for 5s!'))

        print("SHUTTING DOWN")
        self.udp.close()
        self.isconnected = False
        print(cl_lightred('Connection is closed!'))
        rclpy.shutdown()

    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        try:
            thread.start_new_thread(self.__send, (cmd,))
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def __send(self, cmd):
        encoded_cmd = cmd.encode() # Encode to bytes
        self.udp.sendto(encoded_cmd, self.client_address)
