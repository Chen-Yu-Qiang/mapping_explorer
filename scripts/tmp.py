#!usr/bin/env python3
'''ros utils'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import csv
import sys

file_name = '/home/ncslaber/realSense/catkin_ws/src/mapping_explorer/scripts'

def cbLaser(msg):
    # print("receive laser!")
    
    with open(file_name + '.csv', 'w') as csvfile: # or w
        writer = csv.writer(csvfile)
        writer.writerow(np.array(msg.ranges) )

    

if __name__ == "__main__":
    print("Python version: ",sys.version)
    rospy.init_node("laserHandler", anonymous=True)
    subLaser = rospy.Subscriber("/scan_filtered", LaserScan, cbLaser)
    
    print("successfully initialized!")
    

    rospy.spin()
