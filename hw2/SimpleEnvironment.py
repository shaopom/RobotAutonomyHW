import numpy
import pylab as pl

class SimpleEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits

        # Save robot current transform
        init_transform = self.robot.GetTransform()
        res = init_transform[:2, 3]

        # try at most 100 times to find one random collision free config
        xs = numpy.random.uniform(lower_limits[0], upper_limits[0], 100)
        ys = numpy.random.uniform(lower_limits[1], upper_limits[1], 100)
        for n in xrange(100):
            # get uniform sampling in 2D configuration space
            config = [xs[n], ys[n]]

            # transform robot to that config in 6D (x,y,z,r,p,y) space
            # only set the x and y in that 4 by 4 transform matrix
            robot_pose = numpy.eye(4)
            robot_pose[:2, 3] = config
            self.robot.SetTransform(robot_pose)

            # if this 2D random config sampling is collision free, then return this 2D config
            if not self.robot.GetEnv().CheckCollision(self.robot):
                res = numpy.array(config)
                break

        # Restore robot transform
        self.robot.SetTransform(init_transform)

        # Return found random configuration
        # If no collision-free configuration found, then return robots position
        return res

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        sconfig = numpy.array(start_config)
        econfig = numpy.array(end_config)
        return numpy.linalg.norm(econfig - sconfig)

    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        pass

    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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

    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

