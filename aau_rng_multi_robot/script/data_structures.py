#!/usr/bin/env python
# -*- coding: utf-8 -*-



from enum import Enum
import math


"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class Point(object):
 
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z   # Assume the use of 2D Point vectors, so the z axis is unnecessary.

    def __add__(self, other):
        return Point((self.x + other.x), (self.y + other.y), (self.z + other.z))

    def __sub__(self, other):
        return Point((self.x - other.x), (self.y - other.y), (self.z - other.z))

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y) and (self.z == other.z) # tuple(self) == tuple(other)
    
    def __ne__(self, other):
        return not (self == other) # tuple(self) == tuple(other)

    def __rmul__(self, r):
        return Point(self.x * r, self.y * r, self.z * r)

    def __mul__(self, other):
        return Point(self.x * other.x, self.y * other.y, self.z * other.z)

    def angle(self):
        return math.atan2(self.y, self.x) 

    def norme(self):
        return math.sqrt((self.x**2) + (self.y**2) + (self.z**2))

    def move_vector(self):
        """
        Return the movement vector of robot toward a goal
        Take care to avoid having a norm equal to 0
        """
        distance = self.norme()
        try:
            x_move = self.x/distance
            y_move = self.y/distance
            z_move = self.z/distance
            return Point(x_move, y_move, z_move)
        except ZeroDivisionError:
            return None
        
    def collinear(self, other):
        return self.x * other.y==self.y * other.x
    
    def orthogoanl(self, other):
        return self * other==0

    def __repr__(self):
        class_name = type(self).__name__
        return "{}({}, {}, {})".format(class_name, repr(self.x), repr(self.y), repr(self.z))
 
    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.z)
      

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

class Beacon(object): 
  
    def __init__(self, htype, src, dst, position):
        self.htype = htype
        self.src_robot = src
        self.dst_robot = dst
        self.position = position

    def __repr__(self):
        return "Beacon({}, {}, {})".format(repr(self.htype), repr(self.src_robot), repr(self.dst_robot), repr(self.position))

    def __str__(self):
        return "({}, {}, {})".format(self.htype, self.src_robot, self.dst_robot, str(self.position))

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

class HelloNeighbor(object):

    def __init__(self, robotID, position, link_type):
        self.robotID = robotID
        self.position = position
        self.link_type = link_type

    def __repr__(self):
        return "HelloNeighbor({} {} {})".format(repr(self.robotID), repr(self.position), repr(self.link_type))

    def __str__(self):
        return "({} {} {})".format(self.robotID, str(self.position), self.link_type)


"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class Packet(object):

    def __init__(self, beacon, size, neighbors):
        self.beacon = beacon
        self.neighborSize = size
        self.neighbors = neighbors

    def __repr__(self):
        return "Packet({}, {}, {})".format(repr(self.beacon), repr(self.neighborSize), repr(self.neighbors))

    def __str__(self):
        return "({}, {}, {})".format(str(self.beacon), self.neighborSize, str(self.neighbors))


"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
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
        return 'Neighbor({}, {}, {}, {}, {}, {}, {}, {})'.format(repr(self.robotID), repr(self.position), repr(self.mark), repr(self.link_type), repr(self.timer), 
                                                                 repr(self.distance), repr(self.neighborList), repr(self.witnessList))

    def __str__(self):
        return '({}, {}, {}, {}, {}, {}, {}, {})'.format(self.robotID, str(self.position), self.mark, self.link_type, self.timer, 
                                                         self.distance, str(self.neighborList), str(self.witnessList))

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class Neighborhood(object):
  
    def __init__(self, robotID, pos, link_type, timer, distance, distance_to_r):
        self.robotID = robotID
        self.position = pos
        self.link_type = link_type
        self.timer = timer
        self.distance = distance
        self.distance_to_robot = distance_to_r

    def __repr__(self):
        return "Neighborhood({}, {}, {}, {}, {}, {})".format(repr(self.robotID), repr(self.position), repr(self.link_type), repr(self.timer), repr(self.distance), 
                                                             repr(self.distance_to_robot))

    def __str__(self):
        return "({}, {}, {}, {}, {}, {})".format(self.robotID, str(self.position), self.link_type, self.timer, self.distance, self.distance_to_robot)


"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class RNGNeighbor(object):
    
    def __init__(self, robotID, pos, timer):
        self.robotID = robotID
        self.position = pos
        self.timer = timer
        
    def __repr__(self):
        return "RNGNeighbor({}, {}, {})".format(repr(self.robotID), repr(self.position), repr(self.timer), repr(self.timer))

    def __str__(self):
        return "({}, {}, {}, {}, {}, {})".format(self.robotID, str(self.position), self.timer)
  
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""      
class LINKType (Enum):
    SYMETRIK = 0
    ASYMETRIK = 1

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class PACKETType (Enum):
    HELLO_PKT = 0
   
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
""" 
class ROBOTNEIGType (Enum):
    NRNG = 0,
    RNG = 1
