#!/usr/bin/env python

import rospy
import math as ma
import os
import re

# write down all the constants for the file

log_file_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/log/robotdata1.log"
MIN_ANG = -ma.pi / 2
MAX_ANG = ma.pi / 2
MIN_RNG = 0.0
MAX_RNG = 5.0
NUM_LASERS = 180
FREQ = 10

class DataReader:

  def __init__(self):
    # define the file path to read the logs from
    self.log_file = log_file_path
    self.my_file = open(self.log_file, "r")

  def parseFile(self, line):
      # all the data is returned in m and sec
      # check for laser data
      if line[0] is 'L':
         line_list = line.split()
         decoded_data =[float(i) for i in (line_list[1:])]
 	 # should use the odom data here as well
         self.laser_data = decoded_data[6:-1]
         self.laser_data[:] = [i / 100 for i in self.laser_data]
         self.odom_data = decoded_data[0:2]
         self.odom_data[:] = [i / 100 for i in self.odom_data]
         self.odom_data.append(decoded_data[2])
         self.rob_laser_data = decoded_data[3:5]
         self.rob_laser_data[:] = [i / 100 for i in self.rob_laser_data]
         self.rob_laser_data.append(decoded_data[5])
	 self.time_stamp = decoded_data[-1]
         return ['L', self.laser_data, self.odom_data, self.rob_laser_data, self.time_stamp]
      
      # check for odom data
      elif line[0] is 'O':
        line_list = line.split()
 	decoded_data =[float(i) for i in (line_list[1:])]
        self.odom_data = decoded_data[:-2]
        self.odom_data[:] = [i / 100 for i in self.odom_data]
        self.odom_data.append(decoded_data[-2])
        self.time_stamp = decoded_data[-1]
        return ['O', self.odom_data, self.time_stamp]


if __name__ == "__main__":
  d = DataReader()
  for line in d.my_file:
    print len(d.parseFile(line))  
