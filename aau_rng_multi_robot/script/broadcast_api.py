#!/usr/bin/env python
# -*- coding: utf-8 -*-

from socket import socket, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR, SO_BROADCAST
from get_netifaces_addresses import NetworkDevice


class BroadcastSocket(object):

    def __init__(self, ifname, port=12345, msg_len=8192):
        self.netDevice = NetworkDevice(ifname)
        self.ip_address = self.netDevice.get_device_address()
        self.broadcast_address = self.netDevice.get_broadcast_address()
        self.port = port
        self.msg_len = msg_len
        self.network = ('<broadcast>', self.port)
        self.send_sock = socket(AF_INET, SOCK_DGRAM)
        self.recv_sock = socket(AF_INET, SOCK_DGRAM)
        
        self.recv_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.send_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        
        self.recv_sock.bind(('', self.port))

    def sender(self, msg):
        self.send_sock.sendto(msg, self.network)

    def receiver(self):
        data, addr = self.recv_sock.recvfrom(self.msg_len)
        return data, addr

    def close(self):
        self.send_sock.close()
        self.recv_sock.close()
        
        
    def __repr__(self):
        return 'BroadcastSocket({0} {1} {2} {3} {4})' .format(__repr__(self.netDevice), __repr__(self.ip_address), __repr__(self.port), __repr(self.broadcast_address))
        
    def __str__(self):
        return '({0} {1} {2} {3} {4})' .format(self.netDevice, self.ip_address, self.port, self.broadcast_address)


