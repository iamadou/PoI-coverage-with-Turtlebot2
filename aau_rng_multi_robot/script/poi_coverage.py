#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from map_navigation import  MapNavigation
from utils import *
from data_structures import Point
from aau_rng_multi_robot.srv import MaxDistance, ShutDown
import math, random
from std_msgs.msg import Float32

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
# Parameters of Point-Of-Interest coverage algorithm
comm_range = 20.0
epsilon_error = 0.5
rng_max_range = 0.0

goal_pose = None
first_pose = None

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def get_poi_param():
    """
    The purpose of this function is to setup PoI algorithm parameters such as the max. communication 
    range (i.e. comm_range), the epsilon tolerated error (i.e., epsilon_error), the Pose of Goal location
    (i.e., (x, y)_goal_pos and the Pose of the robot initial position (i.e., (x, y)_first_pose) after 
    leaving the docker Pose. This is learned from launch file
    """
    global comm_range
    global epsilon_error
    global goal_pose, first_pose

    try:
        comm_range = rospy.get_param("~comm_range", 20.0)
        epsilon_error = rospy.get_param("~epsilon_error_distance", 0.5)

        x_goal_pose = rospy.get_param("~x_goal_pose")
        y_goal_pose = rospy.get_param("~y_goal_pose")
        goal_pose = Point(x_goal_pose, y_goal_pose, 0.0)

        x_first_pose = rospy.get_param("~x_first_pose")
        y_first_pose = rospy.get_param("~y_first_pose")
        first_pose = Point(x_first_pose, y_first_pose, 0.0)

    except KeyError:
        rospy.logwarn("KeyError exception inside get parameter")
    finally:
        print("Goal {} First pose {}" .format(goal_pose, first_pose))
        rospy.loginfo("Set Parameter without any exception")

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
class PoICoverage (object):

    def __init__(self, comm_range, goal_pose, first_pose, epsilon_error):
        self.robot_range = comm_range
        self.goal_pose = goal_pose
        self.first_pose = first_pose
        self.epsilon_error = epsilon_error
        self.roll = self.pitch = self.yaw = 0
        self.rng_max_distance = 0.0
        self.robot_pose = None
        self.navigator = MapNavigation()

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)  

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.PoseCallback)
        
        rospy.wait_for_service('aau_rng_multi_robot_service')
        self.rng_max_dist = rospy.ServiceProxy('aau_rng_multi_robot_service', MaxDistance)

        rospy.wait_for_service('aau_rng_multi_robot_shutdown')
        self.shutdown_service = rospy.ServiceProxy('aau_rng_multi_robot_shutdown', ShutDown)
        

        rospy.Subscriber('rng_robot_distance', Float32, self.RNGDistanceCallback)
        
    def PoseCallback(self, pose_data):
        rospy.loginfo("[PoseCallback] Robot pose: {}".format(self.robot_pose))
        pose = pose_data.pose
    
        self.robot_pose = Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

        orientation_q = pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        rospy.loginfo("[PoseCallback] Robot pose: {} and Yaw: {}".format(self.robot_pose, self.yaw))

    def RNGDistanceCallback(self, msg):
        self.rng_max_distance = msg.data
        
    def go_to_first_goal(self):
        # Go to the first goal position
        position = {'x': self.first_pose.x, 'y': self.first_pose.y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to ({0}, {1}) pose".format(position['x'], position['y']))
        success = self.navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("First target pose is reached")
        else:
            rospy.loginfo("Robot failed to reach the disired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
       
    def get_rng_max_distance(self):
        try:
            result = self.rng_max_dist()
            self.rng_max_distance = result.rngMaxDistance
            return result.rngMaxDistance
        except rospy.ServiceException, ex:
            rospy.loginfo("Service call failed with error code {}".format(ex))
            return self.rng_max_distance

    def go_to_goal(self):
        previous_pose = Point(0, 0, 0)
        while not rospy.is_shutdown():
           rospy.loginfo("[GO_TO_GOAL] Robot pose {} and goal pose {}".format(self.robot_pose, self.goal_pose))
           rospy.loginfo("Service Call: {}".format(self.get_rng_max_distance()))
           travel_distance = get_travel_distance(self.robot_range, self.get_rng_max_distance()) # self.get_rng_max_distance()
           vect_direction = (self.goal_pose - self.robot_pose)

           if travel_distance < get_distance(self.goal_pose, self.robot_pose):
              next_pose = get_next_target(self.robot_pose, vect_direction, travel_distance)
           else:
              travel_distance = get_distance(self.goal_pose, self.robot_pose) # - random.random() * self.epsilon_error
              next_pose = get_next_target(self.robot_pose, vect_direction, travel_distance)

           if (previous_pose != next_pose):
               position = {'x': next_pose.x, 'y': next_pose.y}
               quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

               rospy.loginfo("Go to ({0}, {1}) pose".format(position['x'], position['y']))    
               success = self.navigator.goto(position, quaternion)
               if success:
                   rospy.loginfo("Hooray, reached the desired pose")
                   previous_pose = next_pose
               else:
                   rospy.loginfo("Robot failed to reach the desired pose")
                   previous_pose = self.robot_pose
                   
               # Sleep to give the last log messages time to be sent
               rospy.sleep(30)

           if get_distance(self.robot_pose, self.goal_pose) <= self.epsilon_error:
               break

        rospy.loginfo("Final Goal is reached")
    
    def shutdown_comm_node(self):
        self.shutdown_service()

    def shutdown(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)

   
"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""
def main():
    global comm_range, epsilon_error, goal_pose, first_pose
    rospy.init_node('poi_coverage')
    get_poi_param()
    rospy.sleep(60)
    rospy.loginfo("PoI node is under instanciation")
    poi = PoICoverage(comm_range, goal_pose, first_pose, epsilon_error)
    rospy.loginfo("PoI node is instanciated")
    poi.go_to_first_goal()
    rospy.sleep(90)
    poi.go_to_goal()
    rospy.sleep(180)
    poi.go_to_first_goal()    
    rospy.spin()

"""
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
# /* ********************************************************************************************************** */
"""

if __name__ == "__main__":
    try:
        main()   
    except rospy.ROSInterruptException:
        rospy.loginfo("PoI node is killed")
