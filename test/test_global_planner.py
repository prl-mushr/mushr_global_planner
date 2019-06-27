import numpy as np
import unittest
import sys

from PIL import Image

sys.path.append('../src') # This is a hack to import the ros-agnostic global planner
from libgp import GlobalPlanner


class GlobalPlannerTestCase(unittest.TestCase):
    def setUp(self):
        # Set up map similar to how ROS MapService does it
        self.raw_map = np.array(Image.open('../maps/real-floor0.pgm'))
        map_data = np.ones_like(self.raw_map, dtype=int)
        map_data[self.raw_map==254] = 0  # 0 is permissible, 1 is not

        params = {}
        self.global_planner = GlobalPlanner(map_data, params)

    def test_one_obstacle(self):
        start_tup = (3550, 1550, 0)
        goal_tup = (3500, 2100, 0)
        success, path = self.global_planner.plan(start_tup, goal_tup, turning_radius=15.0, planning_time=5.0)
        self.assertTrue(success)
        self.showPath(path)

    def test_around_hallway(self):
        start_tup = (4000, 2700, -0.5 * np.pi)
        goal_tup = (3750, 2700, 0)
        success, path = self.global_planner.plan(start_tup, goal_tup, turning_radius=15.0, planning_time=20.0)
        self.assertTrue(success)
        self.showPath(path)

    def test_across_map(self):
        start_tup = (700, 1400, 0)
        goal_tup = (5475, 3025, 0)
        success, path = self.global_planner.plan(start_tup, goal_tup, turning_radius=15.0, planning_time=30.0)
        self.assertTrue(success)
        self.showPath(path)
        
    # Convenience function for plotting result of planning task
    def showPath(self, states):
        map_img = self.raw_map.copy()
        lastState = None
        for state in states:
            y = int(state[1])
            x = int(state[0])
            # draw box for state
            node_size = 5
            for i in range(y - node_size, y + node_size + 1):
                for j in range(x - node_size, x + node_size + 1):
                    map_img[i, j] = 0
            # draw path from last state as series of small boxes
            if lastState is not None:
                begin_y = lastState[1]
                begin_x = lastState[0]
                num_dots = 5
                for dot in range(num_dots):
                    y_dot = int((y - begin_y) * dot/num_dots + begin_y)
                    x_dot = int((x - begin_x) * dot/num_dots + begin_x)
                    edge_node_size = 3
                    for i in range(y_dot - edge_node_size, y_dot + edge_node_size + 1):
                        for j in range(x_dot - edge_node_size, x_dot + edge_node_size + 1):
                            map_img[i, j] = 0
            lastState = state
        im = Image.fromarray(map_img)
        im.show()
       

