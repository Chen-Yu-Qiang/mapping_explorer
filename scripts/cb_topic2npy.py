#!usr/bin/env python3
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import csv

file_path = '/home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/0906_demo_data/demo/'

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return

class Synchronize:
    def __init__(self):
        self.msgColor = None
        self.msgDepth = None
        self.flagColor = False
        self.flagDepth = False
        self.flagSaved = False
    def colorIn(self, color):
        self.msgColor = color
        self.flagColor = True
    def depthIn(self, depth):
        self.msgDepth = depth
        self.flagDepth = True
        
    def save(self):
        if (self.flagDepth and self.flagColor) == True:
            self.imgColor = msg2CV(self.msgColor)
            self.imgDepth = msg2CV(self.msgDepth)
            np.save(file_path + "depth", self.imgDepth) # 1tree, 2tree, 2trees
            np.save(file_path + "color", self.imgColor)
            print("saved!")
            self.flagSaved = True
            
        else: 
            print("haven't receive one of them!")

synchronizer = Synchronize()
def cbDepth(msg):
    if synchronizer.flagSaved == False:
        print("receive depth!")
        synchronizer.depthIn(msg)
        synchronizer.save()
def cbColor(msg):
    if synchronizer.flagSaved == False:
        synchronizer.colorIn(msg)

class Synchronize_pose:
    def __init__(self):
        self.msgIMU = None
        self.msgGPS = None
        self.flagIMU = False
        self.flagGPS = False
        self.flagSaved = False
    def imuIn(self, yaw_degree):
        self.msgIMU = yaw_degree
        self.flagIMU = True
    def gpsIn(self, lat_lon):
        self.msgGPS = lat_lon
        self.flagGPS = True
        
    def save(self):
        if (self.flagIMU and self.flagGPS) == True:
            yaw_rad = self.msgIMU/180*np.pi
            with open(file_path + 'cb_pose.csv', 'w') as csvfile: # or w
                writer = csv.writer(csvfile)
                writer.writerows([[yaw_rad],[self.msgGPS.latitude],[self.msgGPS.longitude]])
            print("saved POSE!")
            self.flagSaved = True
            
        else: 
            print("pose haven't receive one of them!")

synchronizer_pose = Synchronize_pose()
def cbIMU(msg):
    if synchronizer_pose.flagSaved == False:
        synchronizer_pose.imuIn(msg.vector.z) 

def cbGPS(msg):
    if synchronizer_pose.flagSaved == False:
        synchronizer_pose.gpsIn(msg) 
        synchronizer_pose.save()

if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    subIMU = rospy.Subscriber("/imu_filter/rpy/filtered", Vector3Stamped, cbIMU)
    subGPS = rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix, cbGPS)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()
