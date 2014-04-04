import numpy
import IPython
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):

    def __init__(self, herb, resolution):

        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')

        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

    def GetSuccessors(self, node_id):

        successors = []
        #  This function looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        env = self.discrete_env
        coords = env.NodeIdToGridCoord(node_id)
        # print("coordinates for node id ("+str(node_id)+") = "+str(coords))

        # Iterate over dimensions, adding and subtracting one grid per dimension
        for x in xrange(0,self.discrete_env.dimension):
            # Slice copy to not alter coords
            newCoord = coords[:]

            # Subtraction neighbor
            newCoord[x] = coords[x] - 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

            # Addition Neighbor
            newCoord[x] = coords[x] + 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

        return successors

    def CheckCollisions(self, node_id):
        
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        # Put robot into configuration
        self.robot.SetActiveDOFValues(config)

        # Check collision with table
        if self.robot.GetEnv().CheckCollision(self.robot, self.table) == True or self.robot.CheckSelfCollision() == True:
            return True
        else:
            return False



    def InBounds(self, coord):

        config = self.discrete_env.GridCoordToConfiguration(coord)

        for x in xrange(0, self.discrete_env.dimension):
            # Check limits of space for each dimension
            if not(config[x] < self.upper_limits[x]-0.0005 and config[x] > self.lower_limits[x]+0.0005):
                return False
        return True


    def ComputeDistance(self, start_id, end_id):
        # computes the distance between the configurations given
        # by the two node ids

        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid = self.discrete_env.NodeIdToGridCoord(end_id)

        dist = 0
        for x in xrange(0, self.discrete_env.dimension):
            dist = dist + (abs(end_grid[x] - start_grid[x]) * self.discrete_env.resolution)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = self.ComputeDistance(start_id, goal_id)

        return cost

