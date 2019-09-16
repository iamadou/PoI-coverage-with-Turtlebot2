#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import copy

from broadcast_api import *
from data_structures import *
from utils import *
from threading import Thread
from robot_data import RobotData

## rospy.Time.now()
/* ********************************************************************************************************** */
/* ********************************************************************************************************** */
/* ********************************************************************************************************** */
/* ********************************************************************************************************** */
# Parameters of each robot node and its neighborhood information table
IFNAME = "wlo1"                              # net. interface from launch
broad = BroadcastSocket(IFNAME)              # Create comm. socket
robotID = broad.netDevice.get_device_id()    # Get robot ID
robotName = "Bob"                            # Robot name from launch
x = y = z = 0                                # robot current possition
roll, pitch, yaw = 0.0                       # robot orientation
hello_period = 1                             # From launch file 
neig_timeout = 2                             # from launch file also
rdata = RobotData(robotName, robotID, hello_period, neig_timeout, Point(x, y, z))

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def get_random_duration():
    global hello_period
    # Calculates a random amount of time for desynchronizing the hello transmission
    # This should be may be adjuste in order to accelerate the steady-state appearance
    return (random.random() * 2 * hello_period) 
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def set_robot_param():
    global IFNAME, robotName, hello_period
    global neig_timeout, x, y, z
    global broad
    IFNAME = rospy.get_param("/ifname")
    robotName = rospy.get_param("/robot_name", "Bob")
    hello_period = rospy.get_param("/hello_period")
    neig_timeout = rospy.get_param("/neig_timeout")
    position = rospy.get_param("/position")
    x, y, z = position['x'], position['y'], position['z']
    
    broad = BroadcastSocket(IFNAME)
    robotID = broad.netDevice.get_device_id()
    rdata = RobotData(robotName, robotID, hello_period, neig_timeout, Point(x, y, z))
    hello_period = get_random_duration()

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */

def neighborhood_timeout(neighborhood):
    global neig_timeout
    
    if (rospy.Time.now() - neighborhood.timer) > (neig_timeout * 0.5):
        return True
        
    return False
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def onehoplist_timeout()
    global rdata, neig_timeout
    
    neighborhoodList = list()
    witnessesList = list()
    
    for elt in rdata.onehopList:
        while len(elt.neighborList) > 0:
            neighborhood = elt.neighborList.pop()
            if not neighborhood_timeout(neighborhood):
                neighborhoodList.append(neighborhood)
            else:
                continue
        elt.neighborList = neighborhoodList
        while len(elt.witnessList) > 0:
            neighborhood = elt.witnessList.pop()
            if not neighborhood_timeout(neighborhood):
                witnessesList.append(neighborhood)
            else:
                continue
        elt.witnessList = witnessesList
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def neighbor_timeout(neighbor):
    global neig_timeout
    if (rospy.Time.now() - neighbor.timer) >= neig_timeout:
        return True
        
    return False
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def rng_timeout(rng):
    global neig_timeout
    if (rospy.Time.now() - rng.timer) >= neig_timeout:
        return True
        
    return False
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def selective_neighbor_deleting():
    global rdata, neig_timeout
    
    onehop_list = [elt for elt in rdata.onehopList if neighbor_timeout(elt)]
    rng_list = [elt for elt in rdata.onehopList if rng_timeout(elt)]
    
    if len(rdata.onehopList) != 0:
        for elt in rdata.onehopList:
            del elt.neighborList[:]
            del elt.witnessList[:]
        del rdata.onehopList[:]
        
    if len(rdata.rngList) != 0:
        del rdata.rngList[:]
    
    rdata.onehopList = copy.deepcopy(onehop_list)
    rdata.rngList = copy.deepcopy(rng_list)
    
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */    
def get_beacon_packet():
    global x, y, z
    global rdata, robotID
    rcoord = Point(x, y, z)
    beacon = Beacon(PACKETType.HELLO_PKT, robotID, "BROADCAST_ADDR", Point(x, y, z))
    
    return beacon
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def get_hello_neighborList():
    global rdata
    neighbor_list = list()
    
    if len(rdata.onehopList) == 0:
        return None
    else:
        for elt in rdata.onehopList:
            h_neighbor = HelloNeighbor(elt.robotID, elt.position, elt.link_type)
            neighbor_list.append(h_neighbor)
    
    return neighbor_list
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def create_hello_packet():
    h_neighbor_tab = get_hello_neighborList()
    size = len(h_neighbor_tab)
    beacon = get_beacon_packet()
    hello_pkt = Packet(beacon, size, h_neighbor_table)
    return hello_pk
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */                
def send_beacon():
    global hello_period, rdata
    global robotID, robotName

    
    while not rospy.is_shutdown():
        # The aim of this part is selectivily deleted an outdated neighbor
        selective_neighbor_deleting()
        onehoplist_timeout()
        
        # Create the hello packet information
        hpacket = create_hello_packet()
        
        # Convert the object to String object
        hpacket_str = convert_obj2_string(hpacket)
        # Send the message to neighborhood
	broad.sender(hpacket_str)
	rdata.hello_tx++
        rospy.sleep(rospy.Duration(hello_period))
        
    rospy.loginfo("[{} ({})] exit from the transmission loop".format(robotName, robotID))

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def check_in_neighbor(hello_neighborList):
    global roborID
    
    for elt in hello_neighborList:
        if elt.robotID == robotID:
            return 1
            
    return 0 
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def add_neighbor(data):
    global rdata, robotID
    global x, y, z
    witness = 0
    
    for elt in rdata.onehopList:
        if elt.robotID == data.beacon.src_robot:
            elt.position.x = data.beacon.position.x
            elt.position.y = data.beacon.position.y
            elt.position.z = data.beacon.position.z 
            if data.neighborSize > 0:
                 if check_in_neighbor(data.neighbors):
                     elt.mark = 1
                     elt.link_type = LINKType.SYMETRIK
                 else:
                     elt.mark = 1
                    elt.link_type = LINKType.ASYMETRIK
            else:
                elt.mark = 0
                elt.link_type = ASYMETRIK
                
            elt.timer = rospy.Time.now()
            elt.distance = get_distance(Point(x, y, z), elt.position)
            
            if data.neighborSize > 0:
                for neig in data.neighbors
                    if neig.neighborID != robotID
                        witness = 0
                        for elt_neig in elt.neighborList:
                            if elt_neig.robotID == neig.robotID:
                                elt_neig.position = neig.position
                                elt_neig.link_type = neig.link_type
                                elt_neig.timer = rospy.Time.now()
                                elt_neig.distance = get_distance(elt.position, neig.position)
                                elt_neig.distance_to_robot = get_distance(Point(x, y, z), neig.position)
                                witness = 1
                                
                        if witness == 0:
                            dist1 = get_distance(elt.position, neig.position)
                            dist2 = get_distance(Point(x, y, z), neig.position)
                            neighborhood =  Neighborhood(neig.robotID, neig.position, neig.link_type, rospy.Time.now(), dist1, dist2)
                            elt.neighborList.append(neighborhood)
            
            
            return                
            
    
    # Otherwise the robot is new one, so add it as new neighbor
    mark = 0
    link_tyme = 1
    if data.neighborSize > 0:
        if check_in_neighbor(data.neighbors):
            mark = 1
            link_type = LINKType.SYMETRIK
        else:
            mark = 1
            link_type = LINKType.ASYMETRIK
    else:
        mark = 0
        link_type = ASYMETRIK
    distance = get_distance(Point(x, y, z), data.beacon.position)
    neighbor = Neighbor(data.beacon.src_robot, data.beacon.position, mark, link_type, rospy.Time.now(), distance)
    
    # Add the new neighbor's one hop list element in its one hop table
    if data.neighborSize > 0:
        for neig in data.neighbors
            if neig.neighborID != robotID
                dist1 = get_distance(data.beacon.position, neig.position)
                dist2 = get_distance(Point(x, y, z), neig.position)
                neighborhood =  Neighborhood(neig.robotID, neig.position, neig.link_type, rospy.Time.now(), dist1, dist2)
                neighbor.neighborList.append(neighborhood)
    
    # Insert the new neighbor and its one hop 
    # neighborhood in the robot One hop table             
    rdata.onehopList.append(neighbor)       

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def search_robot(robot_id):
    global rdata
    found = 0
    robot_link_type = 0
    
    for neighbor in rdata.onehopList:
        if robot_id == neighbor.robotID:
            robot_link_type = neighbor.link_type
            found = 1
            break
            
    if found == 1 and robot_link_type == LINKType.SYMETRIC:
        return 1
        
    return 0
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def set_witness_node():
    global rdata
    temoin = 0
    
    for neighbor in rdata.onehopList:
        for neighborhood in neighbor.neighborList:
            if search_robot(neighborhood.robotID) == 1:
                temoin = 0
                for witnelt in neighbor.witnessList:
                    if witnelt.robotID == neighborhood.robotID:
                        witnelt.position = neighborhood.position
                        witnelt.link_type = neighborhood.link_type
                        witnelt.timer = neighborhood.timer
                        witnelt.distance = neighborhood. distance
                        witnelt.distance_to_robot = neighborhood.distance_to_robot
                        temoin = 1
                        break
                        
                if temoin == 0:
                    neighborhood = Neighborhood(neighborhood.robotID, neighborhood.position, neighborhood.link_type, neighborhood.timer, neighborhood.distance, neighborhood.distance_to_robot)        
                    neighbor.witnessList.append(neighborhood)

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */

/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def set_mark(robot_id):
    global rdata
    
    for neighbor in rdata.onehopList:
        if neighbor.robotID == robot_id:
            neighbor.mark = -1
            break
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def update_rng():
    global rdata
    temoin = 0
    
    for neighbor in rdata.onehopList:
        if neighbor.link_type == SYMETRIK and neighbor.mark == 1:
            for rng in rdata.rngList:
                if rng.robotID == neighbor.robotID:
                    rng.position = neighbor.position
                    rng.timer = neighbor.timer
                    temoin = 1
                    break
            
            if temoin == 0:
                rng = RNGNeighbor(neighbor.robotID, neighbor.position, neighbor.timer)
                rdata.rngList.append(rng)
                break
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */                   
def rng_neighbor():
    global rdata
    
    for neighbor in rdata.onehopList:
        if len(neighbor.witnessList) > 0:
            for neighborhood in neighbor.witnessList:
                dist1 = neighbor.distance
                dist2 = neighborhood.distance_to_robot
                if search_robot(neighborhood.robotID) == 1 and dist1 > dist2:
                    neighbor.mark = -1
                    break 
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def process_incoming_beacong(data):
    add_neighbor(data)
    set_witness_node()
    
    rng_neighbor()
    update_rng()
    
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def receive_beacon():
    global robotID, robotName

    while not rospy.is_shutdown():
        data, addr = broad.receiver()
        if addr[0] != broad.ip_address:
            process_beacon_msg(convert_string2_obj(data))
            rdata.hello_rx++

    
    rospy.loginfo("[{} ({})] exit from the receiver loop".format(robotName, robotID))
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */        
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */    
def update_pose_callback(msg):
    global x, y, z
    global roll, pitch, yaw
    
    x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
    
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
def main():
    global IFNAME 
    global robotID
    global robot
    global x, y, z
    
    rospy.init_node('aau_rng_communication', anonymous=True)
    set_robot_param()
    
    
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    
    
    thread_send = Thread(target=send_beacon)
    thread_recv = Thread(target=receive_beacon)    

    thread_send.start()
    
    thread_recv.start()

    rospy.spin()
    thread_send.join()
    thread_recv.join()
    rospy.loginfo("After killing the node")


/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
/* ********************************************************************************************************* */
if __name__ == "__main__":
    try:
        main()   
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
