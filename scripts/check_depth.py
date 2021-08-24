#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

def cbPCL(msg):
    # a = np.asarray(msg.data)
    # print(a.shape)
    # print(type(a))
    dist = 1
    pc = ros_numpy.numpify(msg)
    print("receive pcl!")
    points=np.zeros((pc.shape[0],pc.shape[1],3))
    points[:,:,0]=pc['x']
    points[:,:,1]=pc['y']
    points[:,:,2]=pc['z']
    np.save("/home/ncslaber/109-2/210824_calibration/pointCloud_"+str(dist)+"m", points)


if __name__ == '__main__':
    rospy.init_node('pcl_check_node')
    pcl_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, cbPCL)
    rospy.spin()
