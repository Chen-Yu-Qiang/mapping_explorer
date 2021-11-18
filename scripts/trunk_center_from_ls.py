#!/usr/bin/env python3
'''ros utils'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped

from mapping_explorer.msg import Trunkset, Trunkinfo

import numpy as np
import math
import sys


def cbLaser(msg):
    print("enter!")
    '''classify different trunk points'''
    obj_dict = {}
    num_objects = 0
    terminator = True
    for i in range(640):
        ls_range = msg.ranges[i]
        if ls_range < 7.0 and not (math.isinf(ls_range)):
            if i == 0:
                num_objects += 1
                obj_dict[num_objects] = []
            theta = msg.angle_max - msg.angle_increment * i
            x_camera = ls_range * np.sin(theta)
            z_camera = ls_range * np.cos(theta)
            obj_dict[num_objects].append([x_camera,z_camera])
            terminator = True
        elif terminator == True:
            terminator = False
            num_objects += 1
            obj_dict[num_objects] = []

    '''circle matching'''
    centre_x_list = []
    centre_y_list = []
    radius_r_list = []
    trunkSet = Trunkset()
    # print('obj: ',num_objects)
    for i in range(1,num_objects+1):
        trunkinfo = Trunkinfo()
        A = []
        # print(obj_dict)
        # print(i)
        if len(obj_dict[i]) == 0:
            continue
        
        x = np.array(obj_dict[i])[:,0]
        y = np.array(obj_dict[i])[:,1]
        A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
        A = np.asarray(A)[0]
        A = A.T
        # print(A.shape)
        if A.shape[0] < 10:
            continue
        
        k = np.linalg.inv(A.T @ A)
        k = k @ A.T
        k = k @ np.ones((k.shape[1],1))
        centre_x = k[0][0]/(-2)
        centre_y = k[1][0]/(-2)
        radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])
        
        trunkinfo.x = round(centre_x, 3)
        trunkinfo.z = round(centre_y, 3)
        trunkinfo.r = round(radius_r, 3)
        trunkSet.aframe.append(trunkinfo)

        print(round(centre_x, 3), round(centre_y, 3), round(radius_r, 3))
        # centre_x_list.append(round(centre_x, 3))
        # centre_y_list.append(round(centre_y, 3))
        # radius_r_list.append(round(radius_r, 3))

    
    pubTrunk.publish(trunkSet)    
    

if __name__ == "__main__":
    print("Python version: ",sys.version)
    rospy.init_node("laserHandler", anonymous=True)
    subLaser = rospy.Subscriber("/scan_filtered", LaserScan, cbLaser)
    pubTrunk = rospy.Publisher("/wow/trunk_info", Trunkset, queue_size=100)
    
    print("successfully initialized!")
    rospy.spin()
    




