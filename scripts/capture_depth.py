#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

file_path = '/home/ncslaber/109-2/210816_NTU_half/'
dist = 'wall2'

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
            np.save(file_path + "depth_"+dist+"m", self.imgDepth)
            np.save(file_path + "color_"+dist+"m", self.imgColor)
            print("saved!")
            
        else: 
            print("haven't receive one of them!")

synchronizer = Synchronize()
def cbDepth(msg):
    global index, file_path, synchronizer
    
    print("receive depth!")
    synchronizer.depthIn(msg)
    synchronizer.save()
    

def cbColor(msg):
    global file_path, synchronizer
    # print("receive color!")
    synchronizer.colorIn(msg)


if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()
