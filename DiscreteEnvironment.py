import numpy

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

    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space

        grid_coords = ConfigurationToGridCoord(config)
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        return config

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #

        coord = [0] * self.dimension
        for x in xrange(0, self.dimension - 1):
            dimRange = self.upper_limits[x] - self.lower_limits[x]
            shiftedVal = (config[x] + 0.000000001)
            if (shiftedVal > self.upper_limits):
                shiftedVal = self.upper_limits
            coord[x] = math.floor(shiftedVal - self.lower_limits[x]) / (dimRange)

        return coord

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space

        config = [0] * self.dimension
        for x in xrange(0, self.dimension - 1):
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
        for x in xrange(0, len(coord)-1):
            if (x > 0):
                node_id = node_id + self.num_cells[x-1]*(x-1)
            node_id = node_id + coord[x]
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        return coord
