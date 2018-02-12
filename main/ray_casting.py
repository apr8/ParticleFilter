
import rospy
import math as ma
import os
import re
import numpy as np
import cv2
import copy

class RayCasting:

  def __init__(self, m_x, m_y, m_theta, step_size, rotation_search, global_map):

    print 'Setting up the distance lookup Table'
    # initialze x, y and theta
    self.m_x = int(m_x)
    self.m_y = int(m_y)
    self.m_th = int(m_theta)
    print 'Table max x:',m_x,'max y:',m_y,'max_th', -m_theta, ' ', m_theta

    # initialize step size
    self.step = step_size

    # rotationn step
    self.rotation_search = int(rotation_search)

    # initialize map
    self.global_map = global_map

    # initialze the distance table
    self.z_required = np.ndarray((m_x, m_y, 2 * m_theta), np.float32)

    # create a distance table
    self.createDistacleTable()

    print 'Successfully Created a Table'

  def createDistacleTable(self):

    print 'Creating Distance Table'

    # iterate over x
    for i in range(0, self.m_x - 1, 1):
      # iterate over y
      for j in range(0, self.m_y - 1, 1):
        # Go inside only if the index has valid probability
        if not (self.global_map[i][j] < 0.75):
          # iterate over theta
          for k in range(-self.m_th, self.m_th, self.rotation_search):
            # calculate distance
            self.z_required.itemset((i, j, k), copy.copy(self.calculateDistance(i, j, k)))
    return

  def calculateDistance(self, x, y, th):

    # keep incrementing by a step size till we reach a obstacle

    # first check if x and y and within the map, loop over it and if we reach an obstacle we break
    x_new = x
    y_new = y

    if self.global_map[int(x_new)][int(y_new)] < 0.75:
      return 0

    while 0 <= int(x_new) < self.m_x and 0 <= int(y_new) < self.m_y:

      # for safety check for obstacle in its own cell. if obstcle present then return
      if self.global_map[int(x_new)][int(y_new)] < 0.75:
	break

      x_new = x_new + self.step * ma.cos(ma.radians(th))
      y_new = y_new + self.step * ma.sin(ma.radians(th))

    # calculate the distance moved
    dx = x_new - x
    dy = y_new - y

    # print for testing
    #print 'pose',x,y,'dist:',(ma.sqrt(pow(dx,2) + pow(dy,2)) / 10.0)

    return (ma.sqrt(pow(dx,2) + pow(dy,2)) / 10.0)

  def queryTable(self, x, y, th):

    # return the distance value that has been stored in the table
    # make sure these are integers
    x = int(x)
    y = int(y)
    th = int(th)

    # TODO: also check if they are within bounds
    return copy.copy(self.z_required[x, y, th])

if __name__ == '__main__':
  rc = RayCasting(800, 800, 180, 0.1, [[0, 0, 0],[1,1,1],[1,1,1]])
