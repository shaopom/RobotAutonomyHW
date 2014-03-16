import numpy

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.env = self.robot.GetEnv()

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        activeDOFIndices = self.robot.GetActiveDOFIndices()
        config = [0] * len(activeDOFIndices)
        
        #
        # TODO: Generate and return a random configuration
        #

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        # Save robot current configuration
        init_config = self.robot.GetActiveDOFValues()
        res = init_config
        samples = []
        
        # try at most 100 times to find one random collision free config
        # Samples is a DOF by 100 matrix. Each row is one joint value with 100 samples.
        for i in xrange(len(activeDOFIndices)):
            samples.append(numpy.random.uniform(lower_limits[i], upper_limits[i], 100))
        
        for n in xrange(100):
            # get the nth column in the samples matrix
            config = [sample[n] for sample in samples]

            self.robot.SetDOFValues(config, activeDOFIndices)

            if not self.env.CheckCollision(self.robot):
                res = config
                break
        # Restore robot transform
        self.robot.SetDOFValues(init_config, activeDOFIndices)

        # Return found random configuration
        # If no collision-free configuration found, then return robots position

        return numpy.array(res)


    
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
        activeDOFIndices = self.robot.GetActiveDOFIndices()

        config_increment = (end_config - start_config)/500

        check_config = start_config

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        # check all the checking points
        for i in xrange(500):
            # check_config is the interpolation point of the start_config and end_config
            # based on the current check point
            check_config = start_config + config_increment * (i + 1)

            # if check_config is out of DOF limits, then return the last successful check_config
            for n in xrange(len(check_config)):
                if check_config[n] < lower_limits[n] or check_config[n] > upper_limits[n]:
                    self.robot.SetDOFValues(start_config, activeDOFIndices)
                    return check_config - config_increment
            
            # always lock the environment first if you want to change the robot configuration
            with self.env:
                # set the robot to the new transfomation check_config to see whether it collides with
                # all the obstacles
                self.robot.SetDOFValues(check_config, activeDOFIndices)

            if self.env.CheckCollision(self.robot):
                with self.env:
                    self.robot.SetDOFValues(start_config, activeDOFIndices)
                return check_config - config_increment
        
        with self.env:       
            self.robot.SetDOFValues(start_config, activeDOFIndices)

        return check_config
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path
