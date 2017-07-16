#!/usr/bin/env  python
# -*- coding: utf-8 -*-

import  socket
import  sys
import  netifaces

import  rospy
import  std_msgs

class Multicast():
    def __init__(self, group_addr, port):
        my_addr='0.0.0.0'
        if_addr="0.0.0.0"

        default_iface = "eth0"
        if default_iface in netifaces.interfaces():
            iface_data = netifaces.ifaddresses(default_iface)
            addr = iface_data.get(netifaces.AF_INET)[0]["addr"]
        else:
            for iface_name in netifaces.interfaces():
                iface_data = netifaces.ifaddresses(iface_name)
                addr = iface_data.get(netifaces.AF_INET)[0]["addr"]

                #ローカルループバックアドレスを省く
                if(addr != "127.0.0.1"):
                    if_addr = addr

        server_address=(my_addr, port)

        self.sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(server_address)

        mreq=socket.inet_aton(group_addr)+socket.inet_aton(if_addr)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.publisher = rospy.Publisher('/raw_referee', std_msgs.msg.String, queue_size=10)

    def recv(self, buf_length):
        buf = self.sock.recv(buf_length)
        self.publisher.publish(std_msgs.msg.String(buf))
        return  buf
