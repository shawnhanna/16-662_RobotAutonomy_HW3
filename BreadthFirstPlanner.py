import Queue

class BreadthFirstPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config):

        plan = []

        queue = Queue.Queue()

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        name = self.planning_env.robot.GetName()

        d_env = self.planning_env.discrete_env
        start_id = d_env.ConfigurationToNodeId(start_config)
        goal_id = d_env.ConfigurationToNodeId(goal_config)

        costs = dict()
        costs[start_id] = 0
        queue.put(start_id)

        cur_id = start_id
        while cur_id != goal_id and queue.empty() == False:

            print(d_env.NodeIdToConfiguration(cur_id))
            succ = self.planning_env.GetSuccessors(cur_id)
            # print(succ)
            for x in succ:
                new_id = x
                if (costs.get(new_id) is None):
                    cost = costs[cur_id] + 1
                    costs[new_id] = cost
                    queue.put(new_id)

            cur_id = queue.get()

        plan.append(goal_config)
        cur_id = goal_id
        while cur_id != start_id:
            cost = costs[cur_id]
            successors = self.planning_env.GetSuccessors(start_id)
            for succ in range(successors):
                successorCost = costs[succ]
                if (successorCost < cost):
                    cur_id = x
                    break
            plan.append(d_env.NodeIdToConfiguration(cur_id))

        plan.append(start_config)

        plan.reverse()

        return plan
