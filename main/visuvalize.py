#!/usr/bin/env python

import ray_casting
import math as ma
import os
import re
import cv2
import numpy as np

# write down all the constants for the file
map_file_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/map/wean.dat"


class Visuvalize:

   def __init__(self, map_file, rotation_search, step_search):
    # map file get
    self.map_file = open(map_file, 'r')

    # read file from dat file
    print "Setting up Visuvalization parameters and Loading map"
    self.readMapFile()

    # create a distance table for the file
    self.distance_table = ray_casting.RayCasting(self.map_size_x / self.resolution, self.map_size_y / self.resolution, 180, step_search, rotation_search, self.global_map)


   def readMapFile(self):

    print 'Reading map file'

    read_map = False
    flag = False
    i = 0

    for line in self.map_file:
      if 'global_mapsize_x' in line:
        line_split = line.split()
        self.map_size_x = int(line_split[-1])
      if 'global_mapsize_y' in line:
        line_split = line.split()
        self.map_size_y = int(line_split[-1])
      if 'resolution' in line:
        line_split = line.split()
        self.resolution = int(line_split[-1])
      if 'autoshifted_x' in line:
        line_split = line.split()
        self.autoshifted_x = int(line_split[-1])
      if 'autoshifted_y' in line:
        line_split = line.split()
        self.autoshifted_y= int(line_split[-1])
      if 'global_map[0]:' in line:
        line_split = line.split()
        self.map_h= int(line_split[-1])
        self.map_w= int(line_split[-2])
        read_map = True
        self.global_map = np.zeros((self.map_h, self.map_w))

      if read_map:
        if flag:
          line_split = line.split()
          decoded_data = [float(j) for j in line_split]
          self.global_map[i,:] = decoded_data
          i = i + 1
        flag = True
    #print 'global map', self.global_map
    self.global_map = np.transpose(self.global_map)
    self.global_map = np.flipud(self.global_map)
    #print 'global_map after', self.global_map
    print "Done Loading the map"
    self.refreshImage()
    #cv2.imshow('image', self.img)

    return

   def visuvalizeParticle(self, pose):

       # using circles to detect particles
       cv2.circle(self.img, (int(pose[1]), int(pose[0])), 3, (0,0,255), -1)
       #cv2.circle(self.img, (100, 10), 3, (0,0,255), -1)
       return

   def visuvalizeLaserDots(self, pose):

       # using circles to detect particles
       cv2.circle(self.img, (int(pose[1]), int(pose[0])), 3, (255,0,255), -1)
       #cv2.circle(self.img, (100, 10), 3, (0,0,255), -1)
       return

   def visuvalizeLaserData(self, pose):

       # using circles to detect particles
       cv2.circle(self.img, (int(pose[1]), int(pose[0])), 3, (255,0,0), -1)
       #cv2.circle(self.img, (100, 10), 3, (0,0,255), -1)
       return

   def visuvalizeLaser(self, pose1, pose2):

        lineThickness = 2
        cv2.line(self.img, ( pose1[1], pose1[0]), (pose2[1], pose2[0]), (0,255,0), lineThickness)
        # Laser Data Range display
        #for i in laser_data:
        #  x = laser_pose[0] + i * ma.cos(laser_pose[2])
        #  y = laser_pose[1] + i * ma.sin(laser_pose[2])
        #  cv2.circle(self.img, (y, x), 1, (0,0,255), -1)

        return

   def refreshImage(self):
    self.img = cv2.cvtColor(np.float32(self.global_map), cv2.COLOR_GRAY2BGR)
    return

if __name__ == '__main__':

  x = Visuvalize(map_file_path, 1, 0.5)
  #x.visuvalizeParticle([400,400, 1.54])
  #cv2.imshow('image', x.img)
  #x.visuvalizeParticle([100,10])
  #cv2.waitKey(0)
