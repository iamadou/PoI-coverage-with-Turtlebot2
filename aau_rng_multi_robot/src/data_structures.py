#!/usr/bin/env python
# -*- coding: utf-8 -*-



from enum import Enum


class Point(object):
 
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

    def __repr__(self):
        return "Point({}, {}, {})".format(repr(self.x), repr(self.y), repr(self.z))
 
    def __str__(self):
        return "({}, {}, {})".format(str(self.x), str(self.y), str(self.z))
      



class Beacon(object): 
  
    def __init__(self, htype, src, dst, position):
        self.htype = htype
        self.src_robot = src
        self.dst_robot = dst
        self.position = position

    def __repr__(self):
        return "Beacon({}, {}, {})".format(repr(self.htype), repr(self.src_robot), repr(self.dst_robot), self.position.__repr__())

    def __str__(self):
        return "({}, {}, {})".format(self.htype, self.src_robot, self.dst_robot, self.position.__str__())




class HelloNeighbor(object):

    def __init__(self, neighborID, position, link_type):
        self.neighborID = neighborID
        self.position = position
        self.link_type = link_type

    def __repr__(self):
        return "HelloNeighbor({} {} {})".format(repr(self.neighborID), self.position.__repr__(), repr(self.link_type))

    def __str__(self):
        return "({} {} {})".format(str(self.neighborID), self.position.__str__(), str(self.link_type))



class Packet(object):

    def __init__(self, beacon, size, neighbors):
        self.beacon = beacon
        self.neighborSize = size
        self.neighbors = neighbors

    def __repr__(self):
        return "Packet({}, {}, {})".format(self.beacon.__repr__(), repr(self.neighborSize), repr(self.neighbors))

    def __str__(self):
        return "({}, {}, {})".format(self.beacon.__str__(), str(self.neighborSize), str(self.neighbors))



class Neighbor(object):
    
    def __init__(self, robotID, pos, mark, link_type, timer, distance):
        self.robotID = robotID
        self.position = pos
        self.mark = mark
        self.link_type = link_type
        self.timer = timer
        self.distance = distance
        self.neighborList = list()
        self.witnessList = list()

    def __repr__(self):
        return 'Neighbor({}, {}, {}, {}, {}, {}, {}, {})'.format(repr(self.robotID), self.position.__repr__(), repr(self.mark), repr(self.link_type), repr(self.timer), 
                                                                 repr(self.distance), self.onehopList.__repr__(), self.witnessList.__repr__())

    def __str__(self):
        return '({}, {}, {}, {}, {}, {}, {}, {})'.format(str(self.robotID), self.position.__str__(), str(self.mark), str(self.link_type), repr(self.timer), 
                                                         str(self.distance), self.onehopList.__str__(), self.witnessList.__str__())


class Neighborhood(object):
  
    def __init__(self, robotID, pos, link_type, timer, distance, distance_to_r):
        self.robotID = robotID
        self.position = pos
        self.link_type = link_type
        self.timer = timer
        self.distance = distance
        self.distance_to_robot = distance_to_r

    def __repr__(self):
        return "Neighborhood({}, {}, {}, {}, {}, {})".format(repr(self.robotID), self.position.__repr__(), repr(self.link_type), repr(self.timer), repr(self.distance), 
                                                             repr(self.distance_to_neighbor))

    def __str__(self):
        return "({}, {}, {}, {}, {}, {})".format(self.robotID, self.position.__str__(), self.link_type, self.timer, self.distance, self.distance_to_neighbor)


class RNGNeighbor(object):
    
    def __init__(self, robotID, pos, timer):
        self.robotID = robotID
        self.position = pos
        self.timer = timer
        
    def __repr__(self):
        return "RNGNeighbor({}, {}, {})".format(repr(self.robotID), self.position.__repr__(), repr(self.timer), repr(self.timer))

    def __str__(self):
        return "({}, {}, {}, {}, {}, {})".format(self.robotID, self.position.__str__(), self.timer)
        

class LINKType (Enum):
    SYMETRIK = 0
    ASYMETRIK = 1


class PACKETType (Enum):
    HELLO_PKT = 0
    
class ROBOTNEIGType (Enum):
    NRNG = 0,
    RNG = 1
