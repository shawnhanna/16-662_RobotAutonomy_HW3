import Queue

class BreadthFirstPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config):

        plan = []

        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        queue = Queue.Queue()

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        d_env = self.planning_env.discrete_env
        start_id = d_env.ConfigurationToNodeId(start_config)
        goal_id = d_env.ConfigurationToNodeId(goal_config)

        costs = dict()
        costs[start_id] = 0
        queue.put(start_id)

        num_expanded = 0

        print("Goal id = "+str(goal_id))

        cur_id = start_id
        found_path = False
        while cur_id != None and found_path == False:

            # print(d_env.NodeIdToConfiguration(cur_id))
            succ = self.planning_env.GetSuccessors(cur_id)
            for new_id in succ:
                num_expanded = num_expanded + 1
                if (costs.get(new_id) is None):
                    cost = costs[cur_id] + d_env.resolution
                    costs[new_id] = cost
                    # print(new_id)
                    queue.put(new_id)

                    # print("cur id "+str(d_env.NodeIdToConfiguration(new_id))+" form id "+str(d_env.NodeIdToConfiguration(cur_id)))
                    if self.visualize:
                       self.planning_env.PlotEdge(d_env.NodeIdToConfiguration(new_id), d_env.NodeIdToConfiguration(cur_id))

                    if (new_id == goal_id):
                        found_path = True
                        print("Found path")


            cur_id = queue.get()

        plan.append(goal_config)
        cur_id = goal_id
        while cur_id != start_id:
            cost = costs[cur_id]
            form_id = cur_id
            successors = self.planning_env.GetSuccessors(cur_id)
            # print("sccessors: "+str(successors))
            for succ in successors:
                successorCost = costs.get(succ)
                if (successorCost < cost and successorCost != None):
                    # print("Better cost")
                    cur_id = succ
                    cost = successorCost

            plan.append(d_env.NodeIdToConfiguration(cur_id))
            if self.visualize:
               self.planning_env.PlotEdge(d_env.NodeIdToConfiguration(cur_id), d_env.NodeIdToConfiguration(cur_id), 'b')

        if self.visualize:
            self.planning_env.ForcePlot()

        plan.append(start_config)

        plan.reverse()

        # f = open('results.txt', 'a')
        # f.write('Nodes Expanded = %d \n' % num_expanded)
        # f.close()

        return plan
