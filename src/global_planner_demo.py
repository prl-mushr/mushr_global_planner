#!/usr/bin/env python

import rospy
import numpy as np

import utils
from gpnode import MushrGlobalPlanner

def demo():
    service_name = 'mushr_global_planner'
    print('waiting for planner service')
    rospy.wait_for_service(service_name)
    print('found planner service')
    mushr_global_planner = rospy.ServiceProxy(service_name, MushrGlobalPlanner)
    start_tup = (3550, 1550, 0)
    goal_tup = (3500, 2100, 0)
    start_pose = utils.posetup_to_rospose(start_tup)
    goal_pose = utils.posetup_to_rospose(goal_tup)
    print('attempting planning')
    response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=1.0, planning_time=30.0)
    print(response)

if __name__ == "__main__":
    demo()
