import numpy
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx]) / resolution)

        # self.test()

    def test(self):
        # print ('resolution = '+str(self.resolution))
        start = [-4.89,-4.2]
        print start
        out = self.ConfigurationToGridCoord(start)
        print out
        print self.GridCoordToConfiguration(out)
        nodeid = self.ConfigurationToNodeId(start)
        print nodeid
        final = self.NodeIdToGridCoord(nodeid)
        print final

    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space

        grid_coords = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(grid_coords)
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space

        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        # print (config)
        coord = [0] * self.dimension
        for x in xrange(0, self.dimension):
            dimRange = self.upper_limits[x] - self.lower_limits[x]
            shiftedVal = config[x] + 0.0000001
            if (shiftedVal >= self.upper_limits[x]):
                shiftedVal = self.upper_limits[x] - 0.000000001
            coord[x] = math.floor((shiftedVal - self.lower_limits[x]) / (dimRange) * self.num_cells[x])

        return coord

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space

        config = [0] * self.dimension
        for x in xrange(0, self.dimension):
            config[x] = self.lower_limits[x] + coord[x] * self.resolution + (self.resolution / 2)
        return config

    def GridCoordToNodeId(self, coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        #
        # The idea here is to have the nodeid be directly related to
        # grid cell number, but with just one dimension instead of 2
        #
        # Add one

        
        node_id = 0
        multiplications = 1
        for i in xrange(0, self.dimension):
            node_id = node_id + coord[i]*multiplications

            multiplications = multiplications * self.num_cells[i]
        #print ("Coordinate is: "+str(coord)+" Node is: "+str(node_id))
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate

        # print(self.num_cells)
        # coord = [0] * self.dimension
        # for p in range(0, self.dimension):
        #     i = (self.dimension-1)- p
        #     print("HERE "+str(i))
        #     mult = 1
        #     for x in xrange(1,i-1):
        #         mult = mult * self.num_cells[x]
        #         print("mult="+str(mult)
        #     coord[i] = math.floor(node_id / mult)
        #     print("nuum="+str(node_id / mult))
        #     node_id = node_id - (coord[i] * self.num_cells[i])
        coord = [0] * self.dimension
        coord[0] = math.floor(node_id % self.num_cells[0])
        coord[1] = math.floor(node_id / self.num_cells[0])
        # coord = [0] * self.dimension
        # multiplications = 1
        # for i in xrange(0, self.dimension):
        #     # coord[i] = math.floor(node_id / multiplications)
        #     multiplications = multiplications * self.num_cells[i]
        #     coord[i] = math.floor(node_id / self.num_cells[i])

        return coord
