import numpy as np
import math
import time
import cv2
import visuvalize
import ray_casting

class MeasurementModel:

    def __init__(self, Map=None, table=None, rr=1):

        #print 'Setting up measurement model'
        self.z_hit = 4                                    # mixture coeff for gaussian
        self.z_short = 0.2                                     # mixture coeff for short
        self.z_max = .020                                       # misture coeff for max reading
        self.z_rand = 1                                      # misture coeff for random noise
        self.Map = Map
        self.var = 50                                           # variance of the gaussian
        self.lamb = .01                                          # lamda parameter for short
        self.max_range = 2000
        self.epsilon = 1                                        # epsilon for max range reading
        self.threshold = 0.25
        self.table = table
        self.rotation_step = rr
        self.laser_relative_pose = 25

    def prob_hit(self, z, z_star):
        """
        Returns the probability of the reading coming from the true obstacle
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from the true obstacle
        """
        if 0<= z <= self.max_range:
            eta = 1.0 / math.sqrt(2 * math.pi * self.var * self.var)
            gauss = lambda x: math.e**(-math.pow(x - z_star, 2) / (2 * self.var * self.var))
            return gauss(z) * eta

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
            eta  = 1 / (1 - math.exp(-self.lamb * z_star))
            short = lambda x: self.lamb*np.exp(-self.lamb*z)
            return short(z) * eta

        else:
            return 0

    def prob_max(self, z):
        """
        Returns the probability of the reading coming from max range

        :param z: Reading
        :return: Probability of the reading coming from max range
        """

        if z >= self.max_range:
            return 1

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

    def convertLaserPose(self,pose):
        return [pose[0] + self.laser_relative_pose * math.cos(pose[2]), pose[1] + self.laser_relative_pose * math.sin(pose[2]), pose[2]]

    def convertPoseToIndex(self, pose):
        """
        Returns the index to search from distance table for correspoding pose

        :params pose: pose of the robot
        :return: corresponding index
        """
        pose = self.convertLaserPose(pose[:])
        pose[0] = int(pose[0] / 10.0)
        if pose[0] >= 800:
            pose[0] = 799
        elif pose[0] < 0:
            pose[0] = 0
        pose[1] = int(pose[1] / 10.0)
        if pose[1] >= 800:
            pose[1] = 799
        elif pose[1] < 0:
            pose[1] = 0

        #pose[2] = math.degrees(pose[2])
        #if pose[2] > 180 :
        #    pose[2] = pose[2] - 360
        #if pose[2] < -180 :
        #    pose[2] = pose[2] + 360
        #if pose[2] < 0 :
        #    pose[2] = 360 + pose[2]

        #pose[2] = round(pose[2] / self.rotation_step)
        #print 'after', pose
        #if pose[2] >= int(360 / self.rotation_step):
        #    pose[2] = int((360 / self.rotation_step) - 1)
        #elif pose[2] < - int(360 / self.rotation_step):
        #    pose[2] =
        return pose

    def correctTh(self, th):
        if th >= int(360 / self.rotation_step):
            th = int((360 / self.rotation_step)) - th
        elif th <= int(360 / self.rotation_step) :
            th = -(int((360 / self.rotation_step)) + th)
        return th


    def ray_trace(self, x, y, th):
      x_new = x
      y_new = y

      #if self.Map[int(x_new)][int(y_new)] < 0.75:
      #  return 0

      while 0 <= int(x_new) < 800 and 0 <= int(y_new) < 800 :

        # for safety check for obstacle in its own cell. if obstcle present then return
        if self.Map[int(x_new), int(y_new)] <= 0.15:
            break

        x_new = x_new + 0.05 * math.cos(math.pi - th)
        y_new = y_new + 0.05 * math.sin(math.pi - th)
        #if x==400 and y==400:
        #  print 'map_values',self.Map[x_new][y_new]
        #  print 'old x and y and th', x, y, th
        #  print 'new x and y and th', x_new, y_new, th

      # calculate the distance moved
      dx = x_new - x
      dy = y_new - y

      # print for testing
      #print 'pose',x,y,'dist:',(ma.sqrt(pow(dx,2) + pow(dy,2)) / 10.0)

      return (math.sqrt(pow(dx,2) + pow(dy,2)) * 10.0)


    def measurement_probability(self, z, x):
        """
        Returns the probability of the reading fromm the current location

        :param z: Reading
        :param x: Position
        :return: Probability of the reading fromm the current location
        """
        # convert to readable angles
        #print x
        pose = self.convertPoseToIndex(x[:])

        # check for the position of particle in map if it is on obstacle
        #if self.Map[pose[0]][pose[1]] < 0:
        #    return 0.00000000000001

        q = 0

        # for testing
        z_test = []
        for k in range(0, 180, self.rotation_step):
            th = pose[2] - math.radians(90 + k)
            print k, pose, th
            z_star = self.ray_trace(pose[0], pose[1], th)
            z_test.append(z_star)
            p = self.z_hit * self.prob_hit(z[k], z_star) + self.z_short * self.prob_short(z[k], z_star) + \
                self.z_max * self.prob_max(z[k]) + self.z_rand * self.prob_rand(z[k])
            q = q + p

        return z_test


if __name__ == "__main__":
    GlobalMap = MapImage().map
    print len(GlobalMap)
    measurement_model = MeasurementModel(GlobalMap)
