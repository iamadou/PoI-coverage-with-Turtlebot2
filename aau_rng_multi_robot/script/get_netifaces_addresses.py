#!/usr/bin/env python
# -*- coding: utf-8 -*-


from socket import socket, AF_INET, SOCK_DGRAM, inet_ntoa
from fcntl import ioctl
from struct import pack
import sys, re



class NetworkDevice(object):

    def __init__(self, ifname="enp0s3"):
        self.ifname = ifname
        self.ip_address = None
        self.ip_broad_adddress = None
        self.device_id = None
        self.device_ip_block = None

    def get_device_address(self):
        sock = socket(AF_INET, SOCK_DGRAM)
        ip_address = inet_ntoa(ioctl(sock.fileno(), 0x8915, pack('256s', self.ifname[:15]))[20:24])
        self.ip_address = ip_address
        sock.close()
        return self.ip_address

    def get_address_block(self):
        return re.split('(.*)\.(.*)\.(.*)\.(.*)', self.ip_address)
      

    def get_broadcast_address(self):
        self.device_ip_block = self.get_address_block()
        self.ip_broad_address = self.device_ip_block[1]+"."+self.device_ip_block[2]+"."+self.device_ip_block[3]+".255"
        return self.ip_broad_address
        

    def get_device_id(self):
        self.device_id = self.device_ip_block[4]
        return self.device_id


    

