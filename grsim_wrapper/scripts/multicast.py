#!/usr/bin/env  python
# -*- coding: utf-8 -*-

import  rospy
import  socket
import  sys
import  netifaces

import  std_msgs

class Multicast():
    def __init__(self, group_addr, port):
        bind_addr = '0.0.0.0'

        # Create a IPv4/UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Avoid error 'Address already in use'.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Construct a membership_request
        membership_request = socket.inet_aton(group_addr) + socket.inet_aton(bind_addr)

        # Send add membership request to socket
        self.sock.setsockopt(socket.IPPROTO_IP, 
                socket.IP_ADD_MEMBERSHIP, membership_request)

        # Bind the socket to an interfaces
        self.sock.bind((bind_addr, port))

        # Set non-blocking receiving mode
        self.sock.setblocking(False)

        self.publisher = rospy.Publisher('raw_vision', std_msgs.msg.String, queue_size=10)


    def recv(self, buf_length):
        try:
            buf = self.sock.recv(buf_length)

        except:
            return  None

        return  buf
        # self.publisher.publish(std_msgs.msg.String(buf))


    def close(self):
        self.sock.close()
