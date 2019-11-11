#!/usr/bin/env python

import rospy
import numpy as np

import utils
from gpnode import MushrGlobalPlanner, GPNode
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

def demo():
    rospy.init_node('mushr_global_planner_demo')
    service_name = 'mushr_global_planner'
    print('waiting for planner service')
    rospy.wait_for_service(service_name)
    print('found planner service')
    mushr_global_planner = rospy.ServiceProxy(service_name, MushrGlobalPlanner)
    start_pose = Pose(Point(-1.887, 26.930,0.0), Quaternion(0,0,0,1)) 
    goal_pose = Pose(Point(41.804, 0.761, 0.0), Quaternion(0,0,0,1))
    response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=5.0, planning_time=30.0)
    print('got result')
    
    

if __name__ == "__main__":
    demo()
