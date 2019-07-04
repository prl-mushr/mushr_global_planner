import numpy as np
from ompl import base as ob
from ompl import geometric as og


class GlobalPlanner:
    def __init__(self, map_data, params):
        self.map_data = map_data
        self.params = params

    def plan(self, start_tup, goal_tup, turning_radius=10, planning_time=30.0):
        def isStateValid(state):
            y = int(state.getY())
            x = int(state.getX())
            if y < 0 or x < 0 or y >= self.map_data.shape[0] or x >= self.map_data.shape[1]:
                return False
            return bool(self.map_data[y, x] == 0)

        space = ob.DubinsStateSpace(turningRadius=turning_radius)

        # Set State Space bounds
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, 0)
        bounds.setHigh(0, self.map_data.shape[1])
        bounds.setLow(1, 0)
        bounds.setHigh(1, self.map_data.shape[0])
        space.setBounds(bounds)

        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
        si.setStateValidityCheckingResolution(0.0005) # Set based on thinness of walls in map
        si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(ob.ObstacleBasedValidStateSampler(si))) 
        si.setup()

        # Set Start and Goal states
        start = ob.State(space)
        start().setX(start_tup[0])
        start().setY(start_tup[1])
        start().setYaw(start_tup[2])
        goal = ob.State(space)
        goal().setX(goal_tup[0])
        goal().setY(goal_tup[1])
        goal().setYaw(goal_tup[2])

        # Set Problem Definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
        optimObj = ob.PathLengthOptimizationObjective(si)
        pdef.setOptimizationObjective(optimObj)

        # Set up planner
        optimizingPlanner = og.BITstar(si)
        optimizingPlanner.setProblemDefinition(pdef)
        optimizingPlanner.setup()

        solved = optimizingPlanner.solve(planning_time)

        def solution_path_to_tup(solution_path):
            result = []
            states = solution_path.getStates()
            for state in states:
                x = state.getX()
                y = state.getY()
                yaw = state.getYaw()
                result.append((x, y, yaw))
            return result

        solutionPath = None
        if solved:
            solutionPath = pdef.getSolutionPath()
            solutionPath.interpolate(1000)
            solutionPath = solution_path_to_tup(solutionPath)
        return bool(solved), solutionPath
