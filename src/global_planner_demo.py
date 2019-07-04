#!/usr/bin/env python

import rospy
import numpy as np

import utils
from gpnode import MushrGlobalPlanner
from geometry_msgs.msg import Pose, PoseArray

def demo():
    rospy.init_node('mushr_global_planner_demo')
    pub_start = rospy.Publisher('mushr_global_planner_start', Pose, queue_size=10, latch=True)
    pub_goal = rospy.Publisher('mushr_global_planner_goal', Pose, queue_size=10, latch=True)
    pub_path=rospy.Publisher('mushr_global_planner_result', PoseArray, queue_size=10, latch=True)
    service_name = 'mushr_global_planner'
    print('waiting for planner service')
    rospy.wait_for_service(service_name)
    print('found planner service')
    mushr_global_planner = rospy.ServiceProxy(service_name, MushrGlobalPlanner)
    start_tup = (3450, 1500, 0)
    goal_tup = (3500, 1500, 0)
    start_pose = utils.posetup_to_rospose(start_tup)
    goal_pose = utils.posetup_to_rospose(goal_tup)
    pub_start.publish(start_pose)
    pub_goal.publish(goal_pose)
    response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=5.0, planning_time=30.0)
    pub_path.publish(response.path)
    print('got result')
    print('spinning...')
    rospy.spin()
    
    

if __name__ == "__main__":
    demo()
