#!/usr/bin/env python3
'''ros utils'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray

from mapping_explorer.msg import Trunkset, Trunkinfo, Correspondence

import library.ICP_correspondences as icp
import library.plot_utils as plot_utils
import numpy as np
import math
import sys
import json
import csv
import time

# count = 0
# file_name = '/home/ncslaber/110-1/211009_allLibrary/front-right/syn_rosbag/found_trunk/'



'''load trunk map in UTM coordinate'''
list_trunkMap = np.load('center_list_all.npy')
lm_x = list_trunkMap[:,0] 
lm_y = list_trunkMap[:,1] 
lm_radi = list_trunkMap[:,2]
robot_utm = np.array( [1,1,0] ) #None
P = np.vstack((lm_x, lm_y))

def transform2utm_from_camera(cX_m_loc, cY_m_loc, robot_utm):
    utm_x_loc_origin, utm_y_loc_origin, imu_yaw = robot_utm
    cX_utm_loc = cX_m_loc*np.cos(imu_yaw)-cY_m_loc*np.sin(imu_yaw) + utm_x_loc_origin
    cY_utm_loc = cX_m_loc*np.sin(imu_yaw)+cY_m_loc*np.cos(imu_yaw) + utm_y_loc_origin

    return cX_utm_loc, cY_utm_loc


def cbLaser(msg):
    start = time.time()
    # global count,file_name
    global P, robot_utm
    # print("enter!")
    '''classify different trunk points'''
    obj_dict = {}
    num_objects = 0
    discrepancy = 0
    terminator = True
    for i in range(640):
        ls_range = msg.ranges[i]
        if ls_range < 7.0 and not (math.isinf(ls_range)):
            
            theta = msg.angle_max - msg.angle_increment * i
            x_camera = ls_range * np.sin(theta)
            z_camera = ls_range * np.cos(theta)

            if discrepancy != 0:
                print(discrepancy, count)

            if i == 0:
                num_objects += 1
                obj_dict[num_objects] = []

            elif discrepancy < 10 and discrepancy != 0:
                num_objects -= 1
                
            obj_dict[num_objects].append([x_camera,z_camera])

            terminator = True
            discrepancy = 0

        elif terminator == True:
            terminator = False
            num_objects += 1
            obj_dict[num_objects] = []
            discrepancy += 1

        elif terminator == False:
            discrepancy += 1
        
    # json_object = json.dumps(obj_dict, indent=4)      
    # with open(file_name+str(count)+'.json', 'a+') as outfile:
    #     outfile.write(json_object)
    # count += 1   

    '''circle matching'''
    trunkSet = Trunkset()
    list_obs_utm_x = []
    list_obs_utm_y = []
    for i in range(1,num_objects+1):
        # match_circle_from_xz(num_objects, obj_dict)
        trunkinfo = Trunkinfo()
        A = []
        if len(obj_dict[i]) == 0:
            continue
        
        x = np.array(obj_dict[i])[:,0]
        y = np.array(obj_dict[i])[:,1]
        A.append(np.array( [-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)] ))
        A = np.asarray(A)[0]
        A = A.T
        
        if A.shape[0] < 10:             
            continue
        
<<<<<<< HEAD
        k = np.linalg.inv( np.dot(A.T, A) ) 
        k = np.dot(k, A.T)
        k = np.dot( k, np.ones((k.shape[1],1)) ) 
=======
        k = np.linalg.inv( np.dot(A.T, A) )
        k = np.dot(k, A.T)
        k = np.dot(k, np.ones((k.shape[1],1)))
>>>>>>> 9559fa413cd1ea2b2b75c0ff173a9cc784453041
        centre_x = k[0][0]/(-2)
        centre_y = k[1][0]/(-2)
        radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])

        obs_utm_x, obs_utm_y = transform2utm_from_camera(centre_x, centre_y, robot_utm)
        list_obs_utm_x.append(obs_utm_x)
        list_obs_utm_y.append(obs_utm_y)
        
        distance = math.hypot(centre_x,centre_y)
        theta = math.atan2(-centre_x,centre_y)
        trunkinfo.d = round(distance, 3)
        trunkinfo.t = round(theta, 3)
        trunkinfo.r = round(radius_r, 3)
        trunkSet.aframe.append(trunkinfo)
    print('found # of trunk: ', len(trunkSet.aframe))
    if len(trunkSet.aframe) > 1:
        ''' find correspondence by ICP '''
        correspondence = Correspondence()
        
        U = np.vstack((list_obs_utm_x, list_obs_utm_y)) 
        correspondence = icp.get_Rt_by_ICP(P,U)
        pubCorres.publish(correspondence)

    pubTrunk.publish(trunkSet) 
    

    print("time cost: ", time.time()-start)

def cbLML(msg):  # stands for landmark localization
    global robot_utm
    robot_utm = np.array( [1,1,0] )

def cbTrunk(msg):
    global count
    print( 'callback!' )
    while( len(msg.aframe) ):
        inAframe = msg.aframe.pop()
        distance = inAframe.d
        theta = inAframe.t
        radius = inAframe.r

        # with open(file_name+str(count)+'.csv', 'a+') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerow([round(distance, 3), round(theta, 3), round(radius, 3)])
        
    

if __name__ == "__main__":
    print("Python version: ",sys.version)
    rospy.init_node("laserHandler", anonymous=True)
    subLaser = rospy.Subscriber("/scan_filtered", LaserScan, cbLaser)
    pubTrunk = rospy.Publisher("/wow/trunk_info", Trunkset, queue_size=1)
    subTrunk = rospy.Subscriber("/wow/trunk_info", Trunkset, cbTrunk)
    sunLankmarkResult = rospy.Subscriber("landmark_z",Float64MultiArray)

    pubCorres = rospy.Publisher("/wow/correspondence_info", Correspondence, queue_size=1)
    
    print("successfully initialized!")
    rospy.spin()
    

