import math as ma
import ray_casting
import measurement_model
import cv2
import numpy
import visuvalize

map_file_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/map/wean.dat"

# log data path
log_path = "/home/baby/gatech_spring18/stat_techniques_robotics/lab/lab_particlefilter/data/log/robotdata1.log"

# distance table rotation step (1,2,3 etc) while creating distacle table
rotation_step = 10
step_search = 0.5

visuvalize = visuvalize.Visuvalize(map_file_path, rotation_step, step_search)

measurement_model = measurement_model.MeasurementModel(visuvalize.global_map, visuvalize.distance_table, rotation_step)

while True:
    print 'Give the pose to search'
    x = float(raw_input("x: "))
    y = float(raw_input("y: "))
    th = float(raw_input("th: "))
    index = measurement_model.convertPoseToIndex([x, y, th])
    print 'index:', index
    distance = measurement_model.table.queryTable(index[0], index[1], index[2])
    x_new = x + distance * ma.cos(ma.radians(index[2] * 10))
    y_new = y + distance * ma.sin(ma.radians(index[2] * 10))
    index_new = measurement_model.convertPoseToIndex([x_new, y_new, th])
    print 'global_map values before', visuvalize.global_map[index[0]][index[1]]
    print 'index_new:', index_new
    print 'distance = ', distance
    print 'global_map values', visuvalize.global_map[index_new[0]][index_new[1]]
    visuvalize.visuvalizeParticle(index[:])
    visuvalize.visuvalizeParticle([x,y])
    visuvalize.visuvalizeParticle(index_new[:])
    visuvalize.visuvalizeParticle([x_new,y_new])
    visuvalize.visuvalizeLaser(index[:], index_new[:])
    cv2.imshow('image', visuvalize.img)
    cv2.waitKey(0)
    visuvalize.refreshImage()
    #for i in range(10):
    #    th1 = 2 * ma.pi / 180 * rotation_step * i
    #    index1 = measurement_model.convertPoseToIndex([x, y, th1])
    #    print 'index:', index1
    #    distance1 = measurement_model.table.queryTable(index1[0], index1[1], index1[2])
    #    #x_new = x + distance * ma.cos(ma.index1[2])
    #    #y_new = y + distance * ma.sin(index1[2])
    #    index_new = measurement_model.convertPoseToIndex([x_new, y_new, th])
    #    print 'index_new:', index_new
    #    print 'distance = ', distance
    #    visuvalize.visuvalizeParticle(index[:])
    #    visuvalize.visuvalizeParticle([x,y])
    #    visuvalize.visuvalizeParticle(index_new[:])
    #    visuvalize.visuvalizeParticle([x_new,y_new])
    #    visuvalize.visuvalizeLaser(index[:], index_new[:])
    #    cv2.imshow('image', visuvalize.img)
    #    cv2.waitKey(0)
    #    visuvalize.refreshImage()




