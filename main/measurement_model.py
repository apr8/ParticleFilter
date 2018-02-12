import numpy as np
import math
import time
import cv2
import visuvalize
import ray_casting
import scipy.integrate as integrate

class MeasurementModel:

    def __init__(self, Map=None, table=None, rr=1):

        #print 'Setting up measurement model'
        self.z_hit = 0.45                                       # mixture coeff for gaussian
        self.z_short = 0.05                                     # mixture coeff for short
        self.z_max = 0.2                                       # misture coeff for max reading
        self.z_rand = 0.3                                      # misture coeff for random noise
        self.Map = Map
        self.var = 40                                           # variance of the gaussian
        self.lamb = 2                                           # lamda parameter for short
        self.max_range = 4000
        self.epsilon = 1                                        # epsilon for max range reading
        self.threshold = 0.25
        self.table = table
        self.rotation_step = rr

    def prob_hit(self, z, z_star):
        """
        Returns the probability of the reading coming from the true obstacle
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from the true obstacle
        """
        if 0<= z <= self.max_range:
            gauss = lambda x: np.exp(-np.power(x - z_star, 2.) / (2 * self.var * self.var))
            return gauss(z)

        else:
            return 0

    def prob_short(self, z, z_star):
        """
        Returns the probability of the reading coming from a random obstacle in front of the robot
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from a random obstacle in front of the robot
        """
        if 0 <= z <= z_star:
            short = lambda x: self.lamb*np.exp(-self.lamb*z)
            return short(z)

        else:
            return 0

    def prob_max(self, z):
        """
        Returns the probability of the reading coming from max range

        :param z: Reading
        :return: Probability of the reading coming from max range
        """

        if self.max_range-self.epsilon <= z <= self.max_range+self.epsilon:
            return 1/(2*self.epsilon)

        else:
            return 0

    def prob_rand(self, z):
        """
        Returns the probability of the reading coming from a random measurement

        :param z: Reading
        :return: Probability of the reading coming from a random measurement
        """

        if 0 <= z <= self.max_range:
            return 1/self.max_range

        else:
            return 0

    def convertPoseToIndex(self, pose):
        """
        Returns the index to search from distance table for correspoding pose

        :params pose: pose of the robot
        :return: corresponding index
        """
        pose[0] = int(pose[0] * 10)
        if pose[0] >= 800:
          pose[0] = 799
        elif pose[0] < 0:
          pose[0] = 0
        pose[1] = int(pose[1] * 10)
        if pose[1] >= 800:
          pose[1] = 799
        elif pose[1] < 0:
          pose[1] = 0
        pose[2] = ( 180 / math.pi ) * pose[2]
        if pose[2] > 180 :
          pose[2] = pose[2] - 360
        pose[2] = int(self.rotation_step * round(pose[2] / self.rotation_step))
        return pose

    def measurement_probability(self, z, x):
        """
        Returns the probability of the reading fromm the current location

        :param z: Reading
        :param x: Position
        :return: Probability of the reading fromm the current location
        """
        # convert to readable angles
        pose = self.convertPoseToIndex(x[:])

        # check for the position of particle in map if it is on obstacle
        if self.Map[pose[0]][pose[1]] < 0:
          return 0.00000000000001

        q = 1


        for k in range(0, 180, self.rotation_step):
            z_star = self.table.queryTable(pose[0], pose[1], pose[2] -90 + k)
            p = self.z_hit * self.prob_hit(z[k], z_star) + self.z_short * self.prob_short(z[k], z_star) + \
                self.z_max * self.prob_max(z[k]) + self.z_rand * self.prob_rand(z[k])
            q = q*p

        return pow(q, 0.4)


if __name__ == "__main__":
    GlobalMap = MapImage().map
    print len(GlobalMap)
    measurement_model = MeasurementModel(GlobalMap)
