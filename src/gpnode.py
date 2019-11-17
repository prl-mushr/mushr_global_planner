#!/usr/bin/env python

import rospy
import numpy as np
import utils

from nav_msgs.srv import GetMap
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseArray, Pose
from mushr_global_planner.srv import MushrGlobalPlanner, MushrGlobalPlannerResponse

from libgp.global_planner import GlobalPlanner

class GPNode:
    def __init__(self, params, name):
        rospy.init_node(name)
        # get map from Map Service
        print('waiting for map')
        map_name = rospy.get_param("~static_map", "static_map")
	rospy.wait_for_service(map_name)
        map_msg = rospy.ServiceProxy(map_name, GetMap)().map
        print('map received')

        params["kernal_size"] = rospy.get_param('~kernal_size')
        params["interpolation_density"] = rospy.get_param('~interpolation_density')
        params["validity_resolution"] = rospy.get_param('~validity_resolution')

        # reshape map into numpy array to pass into global_planner
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.global_planner = GlobalPlanner(map_data, params)

        # store map metadata for conversion from ros --> numpy
        self.map_metadata = map_msg.info

        self.service = rospy.Service('mushr_global_planner', MushrGlobalPlanner, self.handle_mushr_global_planner)
        self.pub_start = rospy.Publisher('mushr_global_planner_start', Pose, queue_size=10, latch=True)
        self.pub_goal = rospy.Publisher('mushr_global_planner_goal', Pose, queue_size=10, latch=True)
        self.pub_path=rospy.Publisher('mushr_global_planner_result', PoseArray, queue_size=10, latch=True)

        # Optimization for running the same plans
        self.reuse_plans = rospy.get_param('~reuse_plans')
        self.path_dict = {}

        print("waiting for service call...")
        rospy.spin()

    def handle_mushr_global_planner(self, request):
        print("Start Position", request.start.position)
        print("Goal Position", request.goal.position)
        self.pub_start.publish(request.start)
        self.pub_goal.publish(request.goal)

        start = utils.rospose_to_posetup(request.start, self.map_metadata)
        goal = utils.rospose_to_posetup(request.goal, self.map_metadata)

        if str(start)+str(goal) in self.path_dict and self.reuse_plans:
            print("Reusing plan from cache")
            print("waiting for service call...")
            return self.path_dict[str(start)+str(goal)]
        else:
            success, path = self.global_planner.plan(start, goal, request.turning_radius, request.planning_time)
            pose_array = PoseArray()
            pose_array.header.frame_id = 'map'
            if success:
                pose_array.poses = []
                for posetup in path:
                    pose_array.poses.append(utils.posetup_to_rospose(posetup, self.map_metadata))
            
            response = MushrGlobalPlannerResponse()
            response.success.data = success
            response.path = pose_array 
            self.pub_path.publish(response.path)
            self.path_dict[str(start)+str(goal)] = response
            print("waiting for service call...")
            return response

if __name__ == "__main__":
    gpnode = GPNode({}, "test_gpnode")
