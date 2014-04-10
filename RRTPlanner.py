import numpy
from RRTTree import RRTTree
import IPython

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        plan.append(goal_config)

        # Set goal parameters
        self.planning_env.SetGoalParameters(goal_config)

        #IPython.embed()

        # Outer loop until plan complete
        while self.planning_env.ComputeSomeDistance(goal_config, self.tree.GetNearestVertex(goal_config)[1]) > epsilon:
            # Generate Random Configuration
            rand_conf = self.planning_env.GenerateRandomConfiguration()

            # Get Nearest Neighbor
            nn_id, nn_pos = self.tree.GetNearestVertex(rand_conf)

            # Attempt to extend from the nearest neighbor to random configurationz
            new_pos = self.planning_env.Extend(nn_pos, rand_conf)
            # print ("Testing Point")

            if new_pos != None:
                # print("New Point Found!")
                # IPython.embed()
                new_id = self.tree.AddVertex(new_pos)
                self.tree.AddEdge(nn_id, new_id)
                if self.visualize:
                    # print("Plotting")
                    self.planning_env.PlotEdge(nn_pos, new_pos)

        #Gen path from start to goal
        p = len(self.tree.vertices) - 1
        path = [p]
        plan = []
        plan.append(goal_config)

        while p != 0:
            p = self.tree.edges.get(p)
            path.append(p)
            plan.append(self.tree.vertices[p])

        path.reverse()
        plan.reverse()

        # Hardcode for testing
        if len(plan) == 1:
            plan.append(goal_config)

        return plan

