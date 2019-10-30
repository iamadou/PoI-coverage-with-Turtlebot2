#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
import time
import random
import copy
import sys

from broadcast_api import *
from data_structures import *
from utils import *
from threading import Thread
from robot_data import RobotData
from aau_rng_multi_robot.srv import *
from aau_rng_multi_robot.msg import Robot, PointM, RecvString
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
# Parameters of each robot node and its neighborhood information table
IFNAME = "wlp2s0"                            # net. interface from launch
broad = BroadcastSocket(IFNAME)              # Create comm. socket
robotID = broad.netDevice.get_device_id()    # Get robot ID
robotName = "sam"                            # Robot name from launch
x = y = z = 0                                # robot current possition
roll = pitch = yaw = 0.0                     # robot orientation
hello_period = 5                             # From launch file 
neig_timeout = 2                             # from launch file also
randomized_period = 0                        # random float value 
rdata = None                                 # RobotData(robotName, robotID, hello_period, neig_timeout, Point(x, y, z))
rng_pub = None                               # Allows to publish the robot's RNG maximum range
isFirst = False                              # Aims to check if the first hello MSG is sent or not?
# It allows us to randomize only the first hello msg transmission period for remaining other, it is 
# periodically performed without randomization.
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_random_duration(hello_period):
    # Calculates a random amount of time for desynchronizing the hello transmission
    # This should be may be adjuste in order to accelerate the steady-state appearance
    return (random.random() * hello_period) 
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def set_robot_param():
    global IFNAME, robotName, hello_period
    global rdata
    global neig_timeout, x, y, z
    global broad, randomized_period

    try:
        IFNAME = rospy.get_param("~interface")
        robotName = rospy.get_param("~robot_name", "sam")
        hello_period = rospy.get_param("~hello_period")
        neig_timeout = rospy.get_param("~neighbor_timeout")
        x = rospy.get_param("~x_docker_pose")
        y = rospy.get_param('~y_docker_pose') 
        z = 0.0 
    except KeyError as ex:
        rospy.logwarn("Exception type {} appear in set robot parameter" .format(ex))
        sys.exit(1)
    finally:
        rospy.loginfo("Without exception")
    
    
    broad = BroadcastSocket(IFNAME)
    robotID = broad.netDevice.get_device_id()
    rdata = RobotData(robotName, robotID, hello_period, neig_timeout, Point(x, y, z))
    randomized_period = get_random_duration(hello_period)

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_beacon_packet():
    global rdata
    beacon = Beacon(PACKETType.HELLO_PKT, rdata.robot_id, "BROADCAST_ADDR", rdata.position)
    return beacon
    
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */  
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def create_hello_packet():
    """
    Creates hello packet message with the robot's one hop list
    """
    global rdata
    size = 0

    hello_neighbor_list = rdata.get_hello_neighborList()
    if hello_neighbor_list is not None: 
        size = len(hello_neighbor_list)
    beacon = get_beacon_packet()
    hello_pkt = Packet(beacon, size, hello_neighbor_list)
    return hello_pkt

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def send_beacon():
    """
    Uses to send the hello MSG periodically each hello_period
    NB: Only, the first hello MSG is randomized. 
    """
    global rdata
    global isFirst
    global randomized_period

    while not rospy.is_shutdown():
        # Create the hello packet information
        hpacket = create_hello_packet()
        # Convert the object to String object
        hpacket_str = convert_obj2_string(hpacket)
        # Send the message to neighborhood
	broad.sender(hpacket_str)
        rospy.loginfo("Robot {0} with ID {1} with position {2} send a beacon message".format(rdata.robot_name, rdata.robot_id, rdata.position))
	rdata.increase_hello_tx()
        if not isFirst: # Aims to check if it is the first hello pkt
            rospy.sleep(rospy.Duration(hello_period + randomized_period))
            isFirst = True
        else:
            rospy.sleep(rospy.Duration(hello_period))
        
    rospy.loginfo("[{} ({})] exit from the transmission loop".format(robotName, robotID))

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

def receive_beacon():
    """
    Calls when the hello MSG is received from other robots
    """
    global rdata, rng_pub

    while not rospy.is_shutdown():
        data, addr = broad.receiver()
        if addr[0] != broad.ip_address:
            rospy.loginfo("Robot: {0} with ID: {1} receiver data from {2}".format(rdata.robot_name, rdata.robot_id, addr[0]))
            rdata.process_incoming_beacong(convert_string2_obj(data))
            distance = rdata.get_max_distance()
            if distance != 0:
                rng_pub.publish(distance)
            
    rospy.loginfo("[{} ({})] exit from the receiver loop".format(robotName, robotID))

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def handle_shutdown_request(req):
    """
    Calls to shutdown the Node
    """
    rospy.signal_shutdown("User request to kill Ad Hoc Comm. node")

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def handle_get_max_distance(req):
    """
    Service call handler for RNG's neighbor maximum range
    """
    global rdata, rng_pub
    max_dist = rdata.get_max_distance()
    return MaxDistanceResponse(max_dist)

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def PoseCallback(pose_data):
    """
    Call to update robot pose when amcl_pose data is received
    """
    global x, y, z, rdata
    global roll, pitch, yaw
    
    pose = pose_data.pose
    
    x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    rdata.position = Point(x, y, z)
    
    orientation_q = pose.pose.orientation
    orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def main():
    global robotID, robotName, rng_pub
    
    rospy.init_node('aau_rng_communication', anonymous=True)
    set_robot_param()
    
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    rng_pub = rospy.Publisher('rng_robot_distance', Float32, queue_size=10)


    maxRNGDistanceService = rospy.Service('aau_rng_multi_robot_service', MaxDistance, handle_get_max_distance)
    shutDownService = rospy.Service('aau_rng_multi_robot_shutdown', ShutDown, handle_shutdown_request)

    
    rospy.loginfo("Robot name : {0} with Id: {1} is started!".format(robotName, robotID))
    thread_send = Thread(target=send_beacon)
    thread_recv = Thread(target=receive_beacon)    

    thread_send.start()
    thread_recv.start()


    rospy.spin()
    thread_send.join()
    thread_recv.join()
    rospy.loginfo("After killing the node")


"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

if __name__ == "__main__":
    try:
        main()   
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
