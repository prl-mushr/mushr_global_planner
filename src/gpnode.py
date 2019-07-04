#!/usr/bin/env python

import rospy
import numpy as np
import utils

from nav_msgs.srv import GetMap
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseArray
from mushr_global_planner.srv import MushrGlobalPlanner, MushrGlobalPlannerResponse

from libgp.global_planner import GlobalPlanner

class GPNode:
    def __init__(self, params, name):
        # get map from Map Service
        map_name = rospy.get_param("~static_map", "static_map")
	rospy.wait_for_service(map_name)
        map_msg = rospy.ServiceProxy(map_name, GetMap)().map

        # reshape map into numpy array to pass into global_planner
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.global_planner = GlobalPlanner(map_data, params)

        rospy.init_node(name)
        self.service = rospy.Service('mushr_global_planner', MushrGlobalPlanner, self.handle_mushr_global_planner)
        rospy.spin()

    def handle_mushr_global_planner(self, request):
        start = utils.rospose_to_posetup(request.start)
        goal = utils.rospose_to_posetup(request.goal)
        success, path = self.global_planner.plan(start, goal, request.turning_radius, request.planning_time)
        pose_array = PoseArray()
        if success:
            pose_array.poses = []
            for posetup in path:
                pose_array.poses.append(utils.posetup_to_rospose(posetup))
        
        response = MushrGlobalPlannerResponse()
        response.success.data = success
        response.path = pose_array 
        return response

if __name__ == "__main__":
    gpnode = GPNode('', "test_gpnode")
