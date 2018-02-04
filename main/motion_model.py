#!/usr/bin/env python

import math as ma
import os
import re
import random

alpha1 = 0.1 
alpha2 = 0.1
alpha3 = 0.1
alpha4 = 0.1

class MotionModel:

  def __init__(self, alpha, init_odom, type_dist):
    # define the required variables
    # odmetry model constants 
    self.alpha = alpha
    self.prev_odom = init_odom
    self.type_dist = type_dist

  # function to correct orientation
  def correctOrientation(self, angle):
    if angle > ma.pi :
      angle = angle - 2 * ma.pi
    if angle < -ma.pi :
      angle = angle + 2 * ma.pi
    if angle <= ma.pi and angle >= -ma.pi : 
      return angle
    else :
      self.correctOrientation(angle)

  # odometry model
  def odomModel(self, curr_odom, prev_pose):
    # calculate the relative motion paramaeters
    rot1 = ma.atan2(curr_odom[1] - self.prev_odom[1] , curr_odom[0] - self.prev_odom[0]) - self.prev_odom[2]
    rot1 = self.correctOrientation(rot1)
    trans = ma.sqrt(ma.pow((curr_odom[1] - self.prev_odom[1]),2) + ma.pow((curr_odom[0] - self.prev_odom[0]),2))
    rot2 = curr_odom[2] - self.prev_odom[2] - rot1
    rot2 = self.correctOrientation(rot2)

    # generate sample based on the model
    rot1_new = rot1 - self.sampleProb(self.alpha[0] * rot1 + self.alpha[1] * trans)
    trans_new = trans -self.sampleProb(self.alpha[2] * trans + self.alpha[3] * (rot1 + rot2))
    rot2_new = rot2 - self.sampleProb(self.alpha[0] * rot2 + self.alpha[1] * trans)

    # calculate the expected pose from the model
    x = prev_pose[0] + trans_new * ma.cos(prev_pose[2] + rot1_new)
    y = prev_pose[1] + trans_new * ma.sin(prev_pose[2] + rot1_new)
    th = prev_pose[2] + rot1_new + rot2_new
    return [x, y, th]

  # samples based on the model
  def sampleProb(self, b) :
    if self.type_dist == 'trinagular_dist' :
      return b * random.uniform(-1,1) * random.uniform(-1,1)      
    else:
      summation = 0
      for i in range(12) :
        summation = summation + random.uniform(-1,1)
      return b * summation / 6
      

if __name__ == "__main__" :
  m = MotionModel([alpha1, alpha2, alpha3, alpha4], [0, 0, 0], 'normal')
  print m.odomModel([1,1,1],[0,0,0])
  print m.alpha
