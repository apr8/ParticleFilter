#!/usr/bin/env python

import math as ma
import os
import re
import random
import motion_model
import read_log_data
import visuvalize
import cv2
import copy
import measurement_model
import multiprocessing
import numpy as np
# default constants for the program

NUM_PARTICLES = 1
# for motion model
alpha = [.0003, .001, .0005, .005]
# map path file
map_file_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/map/wean.dat"

# log data path
log_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/log/robotdata1.log"

# distance table rotation step (1,2,3 etc) while creating distacle table
rotation_step = 5
step_search = 0.5

z_test = []
# query step distance table for measurement model
class Particle:

  def __init__(self, state, coef, w, global_map, table, rotation_step):

    # initialize pose
    self.pose = state

    # weight of the particle
    self.weight = w

    # initialize the motion model and mesurement model
    self.motion_model = motion_model.MotionModel(coef, [0, 0, 0], 'normal')

    # TODO: Add measeurement model
    self.measurement_model = measurement_model.MeasurementModel(global_map, table, rotation_step)

  def propagateParticle(self, curr_odom):
    # motion model progates for each particle
    # add sampler with map
    self.pose = self.motion_model.odomModel(curr_odom[:], self.pose[:])
    return

  def updateParticle(self, curr_laser):
    # TODO: sensor model updates the weight
    z_test = copy.copy(self.measurement_model.measurement_probability(curr_laser[:], self.pose[:]))
    return z_test

class ParticleFilter:

  def __init__(self, map_file, num, alpha, log_file_path, r_step, s_search):

    print 'Intializing particle filter'
    # TODO: load the map first
    self.visuvalize = visuvalize.Visuvalize(map_file, r_step, s_search)
    self.rotation_step = int(r_step)

    # initialize set of particles
    self.particles = []

    # read logs and based on the data propogate or update
    self.d = read_log_data.DataReader(log_file_path)

    # number of particles
    self.num = num

    # alphas for motion  model
    self.alpha = alpha

    # initial odom
    self.initial_odom = np.array([0, 0, 0])

    # check if the robot has moved to do resmapling
    self.isMoved = False

  def convertImgPosesToDistance(self, pose):
    self.pose_orig = [pose[0] * 10.0, pose[1] * 10.0, pose[2]]
    return

  def moveAndUpdate(self, i, log_odom, log_laser):
    self.particles[i].propagateParticle(log_odom)
    self.z_test = self.particles[i].updateParticle(log_laser)
    return


  def createInitialParticles(self):
    # initialze by creating n particles
    # TODO: generate samples only in free space based on the map

    print 'Creating initial set of particles'

    for i in range(self.num):
      pose = [0, 0, 0]

      while not (self.visuvalize.global_map[int(pose[0]), int(pose[1])] == 1) :
        #pose[0] = random.uniform(0 , 800)
        #pose[1] = random.uniform(0 , 800)
        pose[0] = 400
        pose[1] = 400
        pose[2] = 0
        #pose[2] = random.uniform(-ma.pi , ma.pi)
        #print 'initial_pose:',pose

      # visuvalize the pose
      self.visuvalize.visuvalizeParticle(pose[:])
      # convert them to meters and radians
      self.convertImgPosesToDistance(pose)
      # store each particle with pose, weight and number
      # TODO: Add desired coefficients
      self.particles.append(Particle(self.pose_orig, self.alpha, 1.0 / self.num, self.visuvalize.global_map, self.visuvalize.distance_table, self.rotation_step))
    cv2.waitKey(1)

  def runParticleFilter(self):

    print 'Running particle filter'

    #for line in self.d.my_file:
    while True:
      x=float(raw_input('x= '))
      y=float(raw_input('y= '))
      th=float(raw_input('th= '))

      ## refresh visuvalization in every step
      self.particles[0].pose = [x,y,th]
      #log_data = self.d.parseFile(line)
      ## if data is odom propagate
      #if log_data[0] == 'O':
      #  print log_data[1]
      #  log_odom = log_data[1]
      #  diff = np.array(log_odom) - self.initial_odom
      #  #print ma.sqrt(diff[0] * diff[0] + diff[1] * diff[1])
      #  if ma.sqrt(diff[0] * diff[0] + diff[1] * diff[1]) > 10:
      #      self.isMoved = True
      #  for i in range(self.num):
      #    self.particles[i].propagateParticle(log_odom)
      ## if data is laser the use measurement model
      #elif log_data[0] == 'L':
      #  #threads = []
      #  log_odom = log_data[2]
      #  log_laser = log_data[1]
      #  log_rob_laser = log_data[3]
      #  print log_odom
      #  diff = np.array(log_odom) - self.initial_odom
      #  if ma.sqrt(diff[0] * diff[0] + diff[1] * diff[1]) > 10:
      #      self.isMoved = True

      #  for i in range(self.num):
      #    #pass
      self.moveAndUpdate(0, [0,0,0], 180 * [1])
      #    #thread = multiprocessing.Process(target = self.moveAndUpdate, args = (i, log_odom, log_laser))
      #    #thread.start()
      #    #thread.join()
      #    #threads.append(thread)
      #    #print 'teset'
      #  #for t in threads:
      #  #  t.join()
      #  if self.isMoved:
      #      print 'resampling'
      #      # renormalize the weights before using it for resampling
      #      self.calculateSumOfWeights()
      #      # TODO: resample step add for loop
      #      self.resampleParticleFilter(log_odom)
      #      self.resetWeight()

      ## visuvalize the particles
      self.visuvalizeParticles()

  def calculateSumOfWeights(self):
    # loop over all the samples to calculate
    pf = copy.copy(self.particles)
    w_sum = 0.0
    for i in range(self.num) :
        w_sum = w_sum + copy.copy(pf[i].weight)

    # normalize these weights
    for i in range(self.num):
      temp = copy.copy(pf[i].weight / w_sum)
      self.particles[i].weight = temp

  def resetWeight(self):
    # reset all the weights to 1 / N
    for i in range(self.num):
      self.particles[i].weight = 1.0 / self.num

  def visuvalizeParticles(self):

    # refresh the image
    self.visuvalize.refreshImage()

    for i in range(self.num):
      pose_new = copy.copy(self.particles[i].pose)
      pose_new[0] = int(pose_new[0] / 10)
      pose_new[1] = int(pose_new[1] / 10)
      self.visuvalize.visuvalizeParticle(pose_new)
      ref_angle = ma.pi / 2.0
      ang = 0
      print 'z_test',self.z_test
      for j in self.z_test:
        x = pose_new[0] + j / 10 * ma.cos(pose_new[2] - ref_angle + ma.radians(ang))
        y = pose_new[0] + j / 10 * ma.sin(pose_new[2] - ref_angle + ma.radians(ang))
        ang = ang + self.rotation_step
        print 'angle=', ma.degrees(pose_new[2] - ref_angle + ma.radians(ang))
        self.visuvalize.visuvalizeLaserDots([int(x),int(y),ang])
        self.visuvalize.visuvalizeLaser([int(x),int(y),ang], pose_new)
    cv2.imshow('image', self.visuvalize.img)
    cv2.waitKey(0)

    return

  def resampleParticleFilter(self, log_odom):
    # after updating reweight each of them based on measurement model
    # resampling step using low variance sampler
    # create a copy of old set
    pf = copy.copy(self.particles)
    # create a new set
    pf_new = []

    # create a random number bet 0 and 1/M
    r = random.uniform(0, 1.0 / self.num)
    c = copy.copy(pf[0].weight)
    i = 0
    for m in range(self.num):
      # draw i with prob proportional to weight
      u = r + m * 1.0 / self.num
      while c < u:
        i = i + 1
        c = c + copy.copy(pf[i].weight)
      #pf_new.append(Particle(pf[i].pose[:], pf[i].motion_model.alpha[:], pf[i].weight, self.visuvalize.global_map, self.visuvalize.distance_table))
      pf_new.append(copy.copy(pf[i]))
    self.particles = copy.copy(pf_new)
    self.initial_odom = np.array(log_odom)
    self.isMoved = False

if __name__ == "__main__":

  # TODO: get all the required coefficients and parameters

  # Create a particle filter
  pf = ParticleFilter(map_file_path, NUM_PARTICLES, alpha, log_path, rotation_step, step_search)

  # initialize a set of particles
  pf.createInitialParticles()

  # run the particle filter
  pf.runParticleFilter()
  cv2.waitKey(0)

