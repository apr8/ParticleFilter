#!/usr/bin/env python

import math as ma
import os
import re
import random
import motion_model 
import read_log_data

class ParticleFilter:
 
  def __init__(self, state, coef, w):

    # initialize pose
    self.pose = state
    
    # weight of the particle
    self.weight = w

    # initialize the motion model and mesurement model
    self.motion_model = motion_model.MotionModel(coef, state, 'normal') 
    
    # TODO: Add measeurement model
    
  def propagateParticleFilter(self, curr_odom):
    # motion model progates for each particle
    # add sampler with map
    self.pose = self.motion_model.odomModel(curr_odom, self.pose)
    return
    
  def updateParticleFilter(self, curr_laser):
    # TODO: sensor model updates the weight
    return

if __name__ == "__main__":
  # TODO: get all the required coefficients and parameters


  # TODO: load the map first

 
  # initialze by creating n particles
  # TODO: generate samples only in free space based on the map
  pf = []
  for i in range(5000):
    x = random.uniform(0 , 80)
    y = random.uniform(0 , 80)
    th = random.uniform(-ma.pi , ma.pi)
    pose = [x, y, th]

    # store each particle with pose, weight and number
    pf.append(ParticleFilter(pose, [1,1,1,1], 1))
    
  # read logs and based on the data propogate or update
  d = read_log_data.DataReader()
  resampling = 'False'
  for line in d.my_file:

    log_data = d.parseFile(line)
    # if data is odom propagate
    if log_data[0] == 'O':
      log_odom = log_data[1]
      for i in range(5000):
        pf[i].propagateParticleFilter(log_odom)
      resampling = 'False'
 
    # if data is laser the use measurement model  
    elif log_data[0] == 'L':
      log_odom = log_data[2]
      log_laser = log_data[1]
      log_rob_laser = log_data[3]
      for i in range(5000):
        pf[i].propagateParticleFilter(log_odom)
        pf[i].updateParticleFilter(log_laser)
      
      resampling = 'True'
      # after updating reweight each of them based on measurement model
  
  # resampling step using low variance sampler
  if resampling is 'True':
    # create a new set 
    pf_new = []
    
    # create a random number bet 0 and 1/M
    r = random.uniform(0, 1 / 3000)
    c = pf[0].weight
    i = 0
    for m in range(5000):
      # draw i with prob proportional to weight
      u = r + m * 1 / 3000
      while u > c:
	i = i + 1
        c = c + pf[i].weight
      pf_new.append(ParticleFilter(pf[i].pose, pf[i].motion_model, pf[i].weight)) 
  pf = pf_new
