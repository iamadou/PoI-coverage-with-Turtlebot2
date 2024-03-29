#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cPickle
from data_structures import *
from data_structures import *
import functools
import os, errno
import math



"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

def convert_obj2_string(cls):
    return cPickle.dumps(cls)

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

def convert_string2_obj(string):
    return cPickle.loads(string)

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_distance(pt1, pt2):
    return abs(math.sqrt(((pt1.x - pt2.x) **2) + ((pt1.y - pt2.y) **2) + ((pt1.z - pt2.z) **2)))

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def synchronous(tlockname):
    """A decorator to place an instance based lock around a method """
    def _wrap(func):
        """Decorator implementation."""
        @functools.wraps(func)
        def _wrapped_f(self, *args, **kwargs):
            """ Function protected by 'rlock' """
            tlock = self.__getattribute__(tlockname)
            if not tlock.acquire(blocking=False):
                err = errno.EWOULDBLOCK
                name = self.__class__.__name__
                raise EnvironmentError(err, os.strerror(err), name)

            try:
                return func(self, *args, **kwargs)
            finally:
                tlock.release()
        return _wrapped_f
    return _wrap

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_theta(dx, dy):
    return math.atan2(dy, dx)
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_travel_distance(comm_range, rng_max_range):
    
    if comm_range < rng_max_range:
        print("Uncontrolled behavior: communication range should always be greater than or equal to maximum RNG neighbor range")
        return 0
    
    max_distance = (comm_range - rng_max_range)/2.0
    return max_distance

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_next_target(current_pose, vect_direction, distance):

    # theta = get_theta(vect_direction.y, vect_direction.x)
    move_vector = vect_direction.move_vector()
    x_next = current_pose.x + move_vector.x * distance
    y_next = current_pose.y + move_vector.y * distance
    z_next = current_pose.y + move_vector.z * distance
    return Point(x_next, y_next, z_next)

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
