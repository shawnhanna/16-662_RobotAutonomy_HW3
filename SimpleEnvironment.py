import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import math
import time

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        self.lastDrawTime = int(round(time.time() * 1000))
        self.drawCount = 1

        self.p = 0

    def GetSuccessors(self, node_id):

        successors = []

        #  looks up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        env = self.discrete_env
        coords = env.NodeIdToGridCoord(node_id)
        # print("coordinates for node id ("+str(node_id)+") = "+str(coords))

        # Iterates over each dimension, adding and subtracting to get to different successors
        for x in xrange(0,self.discrete_env.dimension):
            newCoord = coords[:]

            # Negative Neighbor
            newCoord[x] = coords[x] - 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

            # Positive Neighbor
            newCoord[x] = coords[x] + 1
            newID = env.GridCoordToNodeId(newCoord)
            if (self.CheckCollisions(newID) == False and self.InBounds(newCoord) == True):
                successors.append(newID)

        return successors

    def CheckCollisions(self, node_id):
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        # Computes Transform to Robot
        x = config[0]
        y = config[1]
        transform = self.robot.GetTransform()
        transform[0][3] = x
        transform[1][3] = y

        # Assigns Transform to Robot and Checks Collision
        self.robot.SetTransform(transform)
        if self.robot.GetEnv().CheckCollision(self.robot,self.table) == True:
            return True
        else:
            return False



    def InBounds(self, coord):

        config = self.discrete_env.GridCoordToConfiguration(coord)

        for x in xrange(0, self.discrete_env.dimension):
            # Checks each dimension against upper and lower limits
            if not(config[x] < self.upper_limits[x]-0.0005 and config[x] > self.lower_limits[x]+0.0005):
                return False
        return True


    def ComputeDistance(self, start_id, end_id):
        # This is a function that
        # computes the distance between the configurations given
        # by the two node ids
        start = self.discrete_env.NodeIdToConfiguration(start_id)
        end = self.discrete_env.NodeIdToConfiguration(end_id)

        # print("Current id is: "+str(start_id)+" Goal id is: "+str(end_id))
        # print("Current is: "+str(start)+" Goal is: "+str(end))



        # Manhattan distance
        dist = abs(end[0] - start[0]) + abs(end[1] - start[1])
        # print("X dist is: "+str(abs(end[0]-start[0]))+ "Y dist is: "+str(abs(end[1]-start[1])))
        # print("Total dist is: "+str(dist))

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        # This function computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')


        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig, color='k'):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color+'.-', linewidth=2.5)

        self.drawCount = self.drawCount + 1
        millis = int(round(time.time() * 1000))
        if((self.lastDrawTime + self.drawCount) < millis):
            self.lastDrawTime = millis
            pl.draw()

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        t0 = time()
        idx = 0
        print "Shortening Path (original length: %d" % len(path)
        while idx < len(path)-1 and time() - t0 < timeout:
            # Check backwrds from goal
            for ridx in xrange(len(path)-1, idx, -1):
                print idx, ridx
                if self.Extend(path[idx], path[ridx]) != None:
                    
                    dist_ab = self.ComputeSomeDistance(path[idx], path[ridx])
                    dist_path_slice = self.ComputePathSliceLength(path, idx, ridx)
                    # If distance between two points is less than distance along path, slice out inbetween
                    if dist_ab < dist_path_slice:
                        # Remove all inbetween if not next to each other
                        # And done
                        if (ridx - idx+1 > 0):
                            path[idx+1:ridx] = []
                            break
            idx += 1
        print "Shorter Path (length: %d" % len(path)

        return path    

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        if (numpy.random.random() < self.p):
            return numpy.array(self.goal_config)
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        
        # Returns random value in range [low, high]
        # p = low + random[0 to 1] * (high - low)
        config[0] = lower_limits[0] + numpy.random.random()*(upper_limits[0] - lower_limits[0]);
        config[1] = lower_limits[1] + numpy.random.random()*(upper_limits[1] - lower_limits[1]);
        
        return numpy.array(config)

    def ComputeSomeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return self.getDistance(end_config-start_config)

    def getDistance(self, offset):
        return numpy.sqrt(sum(offset**2))

    def checkConfigurationCollision(self, config):
        # This could be extended to N bodies check using loops, quadtrees etc.
        # But this has been solved in openrave etc. so I'm just hardcoding it here.
        tform = numpy.array([[1, 0, 0, config[0]], 
                             [0, 1, 0, config[1]], 
                             [0, 0, 1, 0], 
                             [0, 0, 0, 1]])
        self.robot.SetTransform(tform)
        return checkAABBCollision(self.robot.ComputeAABB(), self.tableAABB)

    def checkRobotTableCollision():
        # This could be extended to N bodies check using loops, quadtrees etc.
        # But this has been solved in openrave etc. so I'm just hardcoding it here.
        return checkAABBCollision(self.robot.ComputeAABB(), self.tableAABB)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        dist = self.ComputeSomeDistance(start_config, end_config)
        MAX_STEP_SIZE = 0.01

        # If two points are so close vs step size, it can make it
        if (dist < MAX_STEP_SIZE):
            return end_config

        num_steps = round(dist/MAX_STEP_SIZE)

        path = numpy.array([numpy.linspace(start_config[0], end_config[0], num_steps),
                            numpy.linspace(start_config[1], end_config[1], num_steps)])

        #IPython.embed()
        for p in path.transpose():
            # print(p)
            if self.checkConfigurationCollision(p):
                return None

        # Return last element in path
        return path.transpose()[-1]