
import math as ma
import os
import re
import numpy as np
import cv2
import copy
import time
import threading
import multiprocessing
#import pdb; pdb.set_trace()


class RayCasting:

    def __init__(
            self,
            m_x,
            m_y,
            m_theta,
            step_size,
            rotation_search,
            global_map):

        print 'Setting up the distance lookup Table'
        # initialze x, y and theta
        self.m_x = int(m_x)
        self.m_y = int(m_y)
        self.m_th = int(m_theta)
        print 'Table max x:', m_x, 'max y:', m_y, 'max_th', -m_theta, ' ', m_theta

        # initialize step size
        self.step = step_size

        # rotationn step
        self.rotation_search = int(rotation_search)

        # initialize map
        self.global_map = global_map

        # time
        self.start_time = time.time()

        # initialze the distance table
        self.z_required = np.ndarray((m_x, m_y, int(2 * m_theta)), np.float32)

        print 'size of distance table =', len(self.z_required)
        print 'search_step = ', self.step
        print 'rotation_search step = ', self.rotation_search

        # create threads list
        self.threads = []
        #np.save('distance_table_new.pyc', self.z_required)

        # create a distance table
        # self.createDistacleTable()

        print 'Successfully Created a  Table'

    def createDistacleTable(self):

        print 'Creating Distance Table'

        # iterate over x
        for i in range(0, self.m_x):
            # iterate over y
            for j in range(0, self.m_y):
                # Go inside only if the index has valid probability
                #self.z_required[i,j,:] =  self.calculateDistanceOverAngles(i,j)
                if (self.global_map[i][j] >= 0.9):
                    #print 'i:',i,'j:',j,'z:',self.z_required[i+j,:]
                    self.calculateDistanceOverAngles(i, j)
                    #thread = RayCastingThread(i, j, self.m_th, self.rotation_search, self.step, self.global_map, self.z_required)
                    #thread = multiprocessing.Process(target = self.calculateDistanceOverAngles, args = (i,j))
                    # self.threads.append(thread)
                    # thread.start()
                    # thread.join()
                    #print 'started', thread.name
                    print i, j
        #print 'Started all the threads'
        # for t in self.threads:
        #    t.join()
        #print 'waiting for threads to finish'
        print(
            "--- %s seconds to create distance table ---" %
            (time.time() - self.start_time))
        np.save('distance_table', self.z_required)
        return

    def calculateDistanceOverAngles(self, x, y):
        z = []
        for k in range(0, 2 * self.m_th, 1):
            # calculate distance
            z.append(self.calculateDistance(x, y, k))

        # if x==400 and y ==400:
        #    print "All the angles for x and y", z[:]
        self.z_required[x, y, :] = z[:]

        return

    def calculateDistance(self, x, y, th):

        # keep incrementing by a step size till we reach a obstacle

        # first check if x and y and within the map, loop over it and if we
        # reach an obstacle we break
        x_new = x
        y_new = y

        # if self.global_map[int(x_new)][int(y_new)] < 0.75:
        #  return 0

        while 0 <= int(x_new) < self.m_x and 0 <= int(y_new) < self.m_y:

            # for safety check for obstacle in its own cell. if obstcle present
            # then return
            if self.global_map[int(x_new)][int(y_new)] <= 0.15:
                break

            x_new = x_new + self.step * ma.cos(ma.radians(th))
            y_new = y_new + self.step * ma.sin(ma.radians(th))

        # calculate the distance moved
        dx = x_new - x
        dy = y_new - y

        # print for testing
        #print 'pose',x,y,'dist:',(ma.sqrt(pow(dx,2) + pow(dy,2)) / 10.0)

        return (ma.sqrt(pow(dx, 2) + pow(dy, 2)) * 10.0)

    def queryTable(self, x, y, th):

        # return the distance value that has been stored in the table
        # make sure these are integers
        x = int(x)
        y = int(y)
        th = int(th)

        # TODO: also check if they are within bounds
        return copy.copy(self.z_required[x, y, th])


if __name__ == '__main__':
    rc = RayCasting(800, 800, 180, 0.1, [[0, 0, 0], [1, 1, 1], [1, 1, 1]])
