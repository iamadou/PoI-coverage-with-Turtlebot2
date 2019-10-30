#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from data_structures import *
from utils import *
from threading import RLock, Timer
import copy


class RobotData(object):
    
    def __init__(self, robot_name, robotID, h_period, neig_timeout, position):
        
        self.rlock = RLock()
        self.robot_id = robotID
        self.hello_period = h_period
        self.neighbor_timeout = neig_timeout
        self.onehopList = list()
        self.robot_degree = 0
        self.rngList = list()
        self.rng_degree = 0
        self.robot_name = robot_name
        self.hello_tx = 0
        self.hello_rx = 0
        self.__position = position

    @synchronous('rlock')
    def increase_hello_tx(self):
        self.hello_tx += 1

    @synchronous('rlock')
    def increase_hello_rx(self):
        self.hello_rx += 1

    @property
    def position(self):
        return self.__position
    
    @position.setter
    def position(self, position):
        self.__position = position


    def __neighborhood_timeout(self):
        if (rospy.Time.now() - neighborhood.timer) >= rospy.Duration(self.neighbor_timeout * 0.5):
            return True
        
        return False


    @synchronous('rlock')
    def onehoplist_timeout(self):
        neighborhoodList = list()
        witnessesList = list()
        rospy.loginfo("[ONEHOP_TIMEOUT] Robot {} with ID {} ({})".format(self.robot_name, self.robot_id, self.__position))
        for neighbor in self.onehopList:
            neighborhoodList = copy.deepcopy(neighbor.neighborList)
            del neighbor.neighborList[:]
            neighbor.neighborList = [elt for elt in neighborhoodList if not self.__neighborhood_timeout(elt)]
            
            witnessesList = copy.deepcopy(neighbor.witnessList)
            del neighbor.witnessList[:]
            neighbor.witnessList = [elt for elt in witnessesList if not self.__neighborhood_timeout(elt)]
       
        if len(neighborhoodList) != 0:
            del neighborhoodList[:]
        if len(witnessesList) != 0:
            del witnessesList[:]

    def __neighbor_timeout(self, neighbor):
        if (rospy.Time.now() - neighbor.timer) >= rospy.Duration(self.neighbor_timeout):
            return True
        
        return False
    

    def __rng_timeout(self, rng):
        if (rospy.Time.now() - rng.timer) >= rospy.Duration(self.neighbor_timeout):
            return True
        
        return False

    @synchronous('rlock')
    def selective_neighbor_deleting(self):
        """
        Cleans the outdating neighbor from one hop neighbor List
        """
        rospy.loginfo("[SELECTIVE_DELETING] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))
        onehop_list = [elt for elt in self.onehopList if not self.__neighbor_timeout(elt)]
        rng_list = [elt for elt in self.rngList if not self.__rng_timeout(elt)]
        rospy.loginfo("[SELECTIVE_DELETING] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, onehop_list, rng_list))
        if len(self.onehopList) != 0:
            for neighbor in self.onehopList:
                del neighbor.neighborList[:]
                del neighbor.witnessList[:]
            del self.onehopList[:]
        
        if len(self.rngList) != 0:
            del self.rngList[:]
    
        if onehop_list is not None:
            self.onehopList = copy.deepcopy(onehop_list)
        if rng_list is not None:
            self.rngList = copy.deepcopy(rng_list)

        rospy.loginfo("[SELECTIVE_DELETING] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))

        
    @synchronous('rlock')
    def get_hello_neighborList(self):
        """
        Provide a list of one hop neighbor of current robot
        """
        rospy.loginfo("[GET_HELLO_NEIGHBOR] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))
        self.selective_neighbor_deleting()
        self.onehoplist_timeout()

        rospy.loginfo("[GET_HELLO_NEIGHBOR] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))
        neighbor_list = list()
        if len(self.onehopList) == 0:
            return None
        
        for neighbor in self.onehopList:
            h_neighbor = HelloNeighbor(neighbor.robotID, neighbor.position, neighbor.link_type)
            neighbor_list.append(h_neighbor)

        rospy.loginfo("[GET_HELLO_NEIGHBOR] Robot {} with ID {} ({}) Onehop {} and RNG {} Return {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList, neighbor_list))
        return neighbor_list


    def __check_in_neighbor(self, hello_neighborList):
        """Allow to check if the robotID is inside the hello neighbor list
           The aims is to find symetric or asymetric neighbor
        """
        if hello_neighborList is None:
            return 0

        if not isinstance(hello_neighborList, list):
            return 0

        for elt in hello_neighborList:
            if elt.robotID == self.robot_id:
                return 1

        return 0

    @synchronous('rlock')
    def __add_neighbor(self, data):
        """
        The aims of this function is to update the neighbor's table
        """
        witness = 0
        rospy.loginfo("[ADD_ROBOT] Robot {} with ID {} ({}) Onehop List neighbors: {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList))
        rospy.loginfo("[ADD_ROBOT] Add Neighbor: Type {0} from src: {1} dst: {2} position: {3}" .format(data.beacon.htype, data.beacon.src_robot, data.beacon.dst_robot, data.beacon.position))
        
        for neighbor in self.onehopList:
            if neighbor.robotID == data.beacon.src_robot:
                neighbor.position.x = data.beacon.position.x
                neighbor.position.y = data.beacon.position.y
                neighbor.position.z = data.beacon.position.z 
                if self.__check_in_neighbor(data.neighbors):
                    neighbor.mark = 1
                    neighbor.link_type = LINKType.SYMETRIK
                else:
                    neighbor.mark = 1
                    neighbor.link_type = LINKType.ASYMETRIK
 
                neighbor.timer = rospy.Time.now()
                neighbor.distance = get_distance(self.__position, neighbor.position)
                rospy.loginfo("\t[ADD_ROBOT] Update robot {} ({}) with Distance {}" .format(neighbor.robotID, neighbor.position, get_distance(self.__position, neighbor.position)))
                if data.neighborSize > 0:
                    for neig in data.neighbors:
                        if neig.robotID != self.robot_id:
                            witness = 0
                            for neighborhood in neighbor.neighborList:
                                if neighborhood.robotID == neig.robotID:
                                    neighborhood.position = neig.position
                                    neighborhood.link_type = neig.link_type
                                    neighborhood.timer = rospy.Time.now()
                                    neighborhood.distance = get_distance(neighbor.position, neig.position)
                                    neighborhood.distance_to_robot = get_distance(self.__position, neig.position)
                                    witness = 1
                                    rospy.loginfo("\t\t[ADD_ROBOT] Update NeighborList robot {} ({}) with Distance {}" .format(neighbor.robotID, neighbor.position, get_distance(self.__position, neighbor.position)))
                                
                            if witness == 0:
                                dist1 = get_distance(neighbor.position, neig.position)
                                dist2 = get_distance(self.__position, neig.position)
                                neighborhood =  Neighborhood(neig.robotID, neig.position, neig.link_type, rospy.Time.now(), dist1, dist2)
                                neighbor.neighborList.append(neighborhood)
                                rospy.loginfo("\t\t[ADD_ROBOT] Update NeighborList robot {} ({}) with Distance {}" .format(neighbor.robotID, neighbor.position, get_distance(self.__position, neighbor.position)))
            
            
                return                
                


        # Otherwise the robot is new one, so add it as new neighbor
        mark = 0
        link_type = 1
        if self.__check_in_neighbor(data.neighbors):
            mark = 1
            link_type = LINKType.SYMETRIK
        else:
            mark = 1
            link_type = LINKType.ASYMETRIK
        
        distance = get_distance(self.__position, data.beacon.position)
        neighbor = Neighbor(data.beacon.src_robot, data.beacon.position, mark, link_type, rospy.Time.now(), distance)
        rospy.loginfo("[ADD_ROBOT] Update robot {} ({}) with Distance {}" .format(neighbor.robotID, neighbor.position, get_distance(self.__position, neighbor.position)))
        # Add the new neighbor of beacon in one hop neighbor list in its one hop table
        if data.neighborSize > 0:
            for neig in data.neighbors:
                if neig.robotID != self.robot_id:
                    dist1 = get_distance(data.beacon.position, neig.position)
                    dist2 = get_distance(self.__position, neig.position)
                    neighborhood =  Neighborhood(neig.robotID, neig.position, neig.link_type, rospy.Time.now(), dist1, dist2)
                    neighbor.neighborList.append(neighborhood)
                    rospy.loginfo("\t\t[ADD_ROBOT] Update NeighborList robot {} ({}) with Distance {}" .format(neighbor.robotID, neighbor.position, get_distance(self.__position, neighbor.position)))
        # Insert the new neighbor and its one hop 
        # neighborhood in the robot One hop table             
        self.onehopList.append(neighbor)  
        rospy.loginfo("[ADD_ROBOT] Robot {} with ID {} ({}) Onehop List neighbors: {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList))   


    def __search_robot(self, robot_id):
        found = 0
        robot_link_type = 0

        for neighbor in self.onehopList:
            if robot_id == neighbor.robotID:
                robot_link_type = neighbor.link_type
                found = 1
                break
            
        if (found == 1) and (robot_link_type == LINKType.SYMETRIK):
            return 1
        
        return 0

    @synchronous('rlock')
    def __set_witness_node(self):
        """
        It's allowed to set the witness neighborhood robot
        """
        rospy.loginfo("[SET_WITNESS] Robot {} with ID {} ({}) Setting witness List {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList))
        temoin = 0
        for neighbor in self.onehopList:
            for neighborhood in neighbor.neighborList:
                if self.__search_robot(neighborhood.robotID) == 1:
                    temoin = 0
                    for witness_elt in neighbor.witnessList:
                        if witness_elt.robotID == neighborhood.robotID:
                            witness_elt.position = neighborhood.position
                            witness_elt.link_type = neighborhood.link_type
                            witness_elt.timer = neighborhood.timer
                            witness_elt.distance = neighborhood.distance
                            witness_elt.distance_to_robot = neighborhood.distance_to_robot
                            temoin = 1
                            break
                        
                    if temoin == 0:
                        neighborhood = Neighborhood(neighborhood.robotID, neighborhood.position, neighborhood.link_type, neighborhood.timer, neighborhood.distance, neighborhood.distance_to_robot)        
                        neighbor.witnessList.append(neighborhood)

        rospy.loginfo("[SET_WITNESS] Robot {} with ID {} ({}) Setting witness List {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList)) 

    @synchronous('rlock')
    def __update_rng(self):
        temoin = 0
        rospy.loginfo("[UPDATE_RNG] Robot {} with ID {} ({}) UPadting RNG List {}".format(self.robot_name, self.robot_id, self.__position, self.rngList))
        for neighbor in self.onehopList:
            temoin = 0
            if neighbor.link_type == LINKType.SYMETRIK and neighbor.mark == 1:
                for rng in self.rngList:
                    if rng.robotID == neighbor.robotID:
                        rospy.loginfo("[UPDATING] Robot with ID: {} is current RNG neighbor".format(rng.robotID))
                        rng.position = neighbor.position
                        rng.timer = neighbor.timer
                        temoin = 1
                        break
            
                if temoin == 0:
                    rng = RNGNeighbor(neighbor.robotID, neighbor.position, neighbor.timer)
                    self.rngList.append(rng)
                    rospy.loginfo("Robot with ID: {} is current RNG neighbor".format(neighbor.robotID))
                    break

        rospy.loginfo("[UPDATE_RNG] Robot {} with ID {} ({}) UPadting RNG List {}".format(self.robot_name, self.robot_id, self.__position, self.rngList))


    @synchronous('rlock')
    def __rng_neighbor(self):
        rospy.loginfo("[RNG_NEIGHBOR] Robot {} with ID {} ({}) Setting RNG mark {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList))
        for neighbor in self.onehopList:
            if len(neighbor.witnessList) > 0:
                for neighborhood in neighbor.witnessList:
                    dist1 = neighbor.distance
                    dist2 = neighborhood.distance_to_robot
                    dist3 = get_distance(neighbor.position, neighborhood.position)
                    if (self.__search_robot(neighborhood.robotID) == 1) and (dist1 > dist2) and (dist1 >dist3):
                        neighbor.mark = -1
                        break 
        rospy.loginfo("[RNG_NEIGHBOR] Robot {} with ID {} ({}) Setting RNG mark {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList))

    @synchronous('rlock')
    def process_incoming_beacong(self, beacon_payload):
        rospy.loginfo("[PROCESS_INCOMING] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))
        self.increase_hello_rx()
        self.__add_neighbor(beacon_payload)
        self.__set_witness_node()
        self.__rng_neighbor()
        self.__update_rng()
        rospy.loginfo("[PROCESS_INCOMING] Robot {} with ID {} ({}) Onehop {} and RNG {}".format(self.robot_name, self.robot_id, self.__position, self.onehopList, self.rngList))

    @synchronous('rlock')
    def get_max_distance(self):
        maxDistance = 0.0
        for elt in  self.rngList:
            if maxDistance < get_distance(elt.position, self.__position):
                maxDistance = get_distance(elt.position, self.__position)
            
        return maxDistance  
        

    def __repr__(self):
        return "RobotData({}, {}, {}, {}, {}, {}, {}, {}, {})" .format(repr(self.robot_id), repr(self.robot_name), repr(self.onehopList), repr(self.robot_degree), repr(self.rngList), 
                                                                       repr(self.rng_degree), repr(self.hello_tx), repr(self.hello_rx), repr(self.position))
                                                                            
    def __str__(self):
        return "({}, {}, {}, {}, {}, {}, {}, {}, {})" .format(self.robot_id, self.robot_name, str(self.onehopList), self.robot_degree, str(self.rngList), self.rng_degree, self.hello_tx, 
                                                              self.hello_rx, str(self.position))
