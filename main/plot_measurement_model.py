import numpy as np
import math
import time
import cv2
#import visuvalize
#import ray_casting
import matplotlib.pyplot as plt
import measurement_model
import visuvalize.py

m = measurement_model.MeasurementModel
OBclass MeasurementModel:

    def __init__(self, Map=None, table=None, rr=1):

        #print 'Setting up measurement model'
        self.z_hit = 4                                    # mixture coeff for gaussian
        self.z_short = 0.2                                     # mixture coeff for short
        # misture coeff for max reading
        self.z_max = .020
        # misture coeff for random noise
        self.z_rand = 1
        self.Map = Map
        self.var = 50                                           # variance of the gaussian
        # lamda parameter for short
        self.lamb = .01
        self.max_range = 2000
        # epsilon for max range reading
        self.epsilon = 1
        self.threshold = 0.25
        self.table = table
        self.rotation_step = rr
        self.laser_relative_pose = 25

        m = np.linspace(0, self.max_range, 0.1)
        z_s = 1000
        p = []
        for i in m:
            p.append(
                self.z_hit *
                self.prob_hit(
                    i,
                    z_s) +
                self.z_short *
                self.prob_short(
                    i,
                    z_s) +
                self.z_max *
                self.prob_max(i) +
                self.z_rand *
                self.prob_rand(i))
        plt.plot(m, p, 'ro')
        plt.show()

    def prob_hit(self, z, z_star):
        """
        Returns the probability of the reading coming from the true obstacle
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from the true obstacle
        """
        if 0 <= z <= self.max_range:
            eta = 1.0 / math.sqrt(2 * math.pi * self.var * self.var)

            def gauss(x): return math.e**(-math.pow(x -
                                                    z_star, 2) / (2 * self.var * self.var))
            return gauss(z) * eta

        else:
            return 0
