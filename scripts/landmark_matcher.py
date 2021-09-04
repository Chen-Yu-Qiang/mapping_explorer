'''math tool'''
import csv
import numpy as np
from scipy.spatial import distance as dist

'''plot tool'''
import matplotlib.pyplot as plt

'''image tool'''
import cv2
import statistics as sta

import utm
from pyproj import Proj
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/test_data/"

''' show raw data '''
npDepth = np.load(file_path+"depth_2tree.npy")
npColor = np.load(file_path+"color_2tree.npy")
npDepthF = cv2.convertScaleAbs(npDepth, alpha=0.02) # 6m
npDepthF_color = cv2.applyColorMap(npDepthF, cv2.COLORMAP_JET)
fig=plt.figure(figsize=(10,10))
subplot = fig.add_subplot(121)
subplot.imshow(npColor)
subplot = fig.add_subplot(122)
subplot.imshow(npDepthF_color)

''' to world coordinate '''
cx_d = 320.6562194824219 #424
cy_d = 241.57083129882812 #241
fx_d = 384.31365966796875 #424
fy_d = 384.31365966796875 #424
npPointX = np.asarray(range(640))-cx_d
npPointX = np.diag(npPointX)
npPointX = npDepth.dot(npPointX)/ fx_d * (-1)

npPointY = np.asarray(range(480))-cy_d
npPointY = np.diag(npPointY)
theta = 0/180*np.pi
npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
npPointY = npPointY*np.cos(theta) + npDepth * np.sin(theta) + 410
npPointY = npPointY.astype('float16')

''' depth segmentation: show layers '''
npHeight = np.copy(npPointY)
color_seq = {'brown': (0,130,210), 'red':(0,0,255),'yellow':(22,220,220),
                'green':(0,255,0), 'blue':(255,0,0),'black':(0,0,0), 
                'gray':(201,201,201), 'orange':(13,101,245), 'sky':(239,178,142) }
npHeight_seg = np.zeros((npDepth.shape[0],npDepth.shape[1],3))

npHeight_seg[npHeight<300]=color_seq['gray']
npHeight_seg[np.logical_and(npHeight<500,npHeight>300)]=color_seq['orange']
npHeight_seg[np.logical_and(npHeight<700,npHeight>500)]=color_seq['green']
npHeight_seg[np.logical_and(npHeight<900,npHeight>700)]=color_seq['yellow']
npHeight_seg[np.logical_and(npHeight<1100,npHeight>900)]=color_seq['blue']
npHeight_seg[np.logical_and(npHeight<1300,npHeight>1100)]=color_seq['red']
npHeight_seg[npHeight>1300]=color_seq['sky']
npHeight_seg[npHeight==410]=color_seq['black']
npHeight_seg = npHeight_seg.astype('uint8')
plt.title('slice every 20 cm', fontsize=15)
plt.imshow(cv2.cvtColor(npHeight_seg, cv2.COLOR_BGR2RGB))

''' top-down view grid '''
def depth_Z(u,v):
    return npDepth[v][u]
height_layer_tmp = np.logical_and(npHeight<500,npHeight>300)
height_layer = np.logical_and(height_layer_tmp,npHeight!=410)

plane_l1 = np.zeros((int(10/0.05),int(10/0.05)),dtype=np.uint8)

row, column = npDepth.shape
scan_height_ = 10
center_y = row/2
offset = int(center_y - scan_height_/2 )

for v in range(row):
    if height_layer[v].any() == True:
        for u in range(column):
            if height_layer[v][u] == True:
                z_depth = depth_Z(u,v)
                if (z_depth>50 and z_depth<6000):
                    x_depth = (u-cx_d)/fx_d*depth_Z(u,v)
                    plane_l1[200-int(z_depth/50)][int(x_depth/50)+100]+=1
                    
hieght_or = np.zeros((int(10/0.05),int(10/0.05)), dtype=np.uint8)
hieght_or[plane_l1>4]=255
subplot = fig.add_subplot(222)
fig, ax = plt.subplots(figsize=(8,8))
plt.title('filter grid > 4', fontsize=15)
plt.imshow(cv2.cvtColor(hieght_or, cv2.COLOR_BGR2RGB))

''' find connected component and push into point array A '''
num_objects, labels = cv2.connectedComponents(hieght_or)
centre_x_list = []
centre_y_list = []
circle_bd = np.zeros(hieght_or.shape, dtype=np.uint8)
print('>>>>num_objects:',num_objects)
for i in range(num_objects):
    A = []
    for x in range(200):
        for y in range(200):
            if labels[x][y] == i+1:
                A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
    A = np.asarray(A)
    print('# of points: ',A.shape)
    if A.shape[0] < 10:
        continue

    k = np.linalg.inv(A.T @ A)
    k = k @ A.T
    k = k @ np.ones((k.shape[1],1))
    centre_x = k[0][0]/(-2)
    centre_y = k[1][0]/(-2)
    radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])
    print('x,y,r: ', int(centre_x+0.5), int(centre_y+0.5), int(radius_r+0.5))
    
    cv2.circle(circle_bd,(int(centre_y+0.5), int(centre_x+0.5)), int(radius_r+0.5), 150, 2)
    
    centre_x_list.append(int(centre_x+0.5))
    centre_y_list.append(int(centre_y+0.5))

    cv2.putText(circle_bd, #numpy array on which text is written
                str(int(centre_x+0.5))+','+str(int(centre_y+0.5)), #text
                (int(centre_y)-20,int(centre_x)-20), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.2, #font size
                255, #font color
                1, cv2.LINE_AA) #font stroke
circle_bd[hieght_or==255]=255
centre_x_list = np.asarray(centre_x_list)
centre_y_list = np.asarray(centre_y_list)
fig,ax = plt.subplots(figsize=(8,8))
plt.xlim((500,700))
plt.ylim((1024,500))
plt.imshow(cv2.cvtColor(circle_bd, cv2.COLOR_BGR2RGB))

''' load robot current pose '''
with open(file_path+'cb_pose.csv', 'r') as csvfile:
    imu_yaw, lat, lng = csv.reader(csvfile, delimiter=',')
imu_yaw = float(imu_yaw[0])
lat = float(lat[0])
lng = float(lng[0])

_, _, zone, R = utm.from_latlon(lat, lng)
proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
utm_x_loc, utm_y_loc = proj(lng, lat)

cX_m_loc = (centre_y_list-100)*0.05
cY_m_loc = (200-centre_x_list)*0.05
cX_utm_loc = cX_m_loc*np.cos(imu_yaw)-cY_m_loc*np.sin(imu_yaw) + utm_x_loc
cY_utm_loc = cX_m_loc*np.sin(imu_yaw)+cY_m_loc*np.cos(imu_yaw)+ utm_y_loc

''' load landmark map '''
directory = '/home/ncslaber/109-2/210725_NTU_leftAreaLibrary/'
bag_name = 'ntu_test3_2021-07-25-18-23-39/'
file_path_map = directory+bag_name
centroid_filteredNP = np.load(file_path_map+'centroid_filteredList.npy')
centroid_filteredList = centroid_filteredNP.tolist()
centroid_filteredNP.shape

center_utm_ref = np.load(file_path_map+'center_utm_ref.npy')

cX_utm_ref = center_utm_ref[0,:]
cY_utm_ref = center_utm_ref[1,:]
plt.scatter(cX_utm_ref, cY_utm_ref, c='g', label='reference landmarks', marker='X',s=100)
# plt.scatter(utm_x_ref, utm_y_ref, label='start_recording',c='black')
plt.axis('equal')
plt.title('global map in UTM', fontsize=15)
plt.legend()
plt.show()

''' find rigid transformation '''
P = np.vstack((cX_utm_ref,cY_utm_ref))
U = np.vstack((cX_utm_loc,cY_utm_loc))

''' plot result '''
traj = np.load('/home/ncslaber/109-2/210725_NTU_leftAreaLibrary/ntu_test2_2021-07-25-17-42-59/traj_GPS_filtered.npy')
traj_x = traj[0,:]
traj_y = traj[1,:]
plt.scatter(cX_utm_ref, cY_utm_ref, c='g', label='ref landmarks', marker='X',s=100)
# plt.scatter(utm_x_ref, utm_y_ref, label='start_recording',c='black')
plt.scatter(cX_utm_loc, cY_utm_loc, label='scanned landmarks',c='b')
plt.scatter(utm_x_loc, utm_y_loc, label='robot pose',c='b', marker="v")
plt.scatter(traj_x[0:len(traj_x)-400:100], traj_y[0:len(traj_y)-400:100],c='b',s=1)

plt.scatter(utm_loc_new[0], utm_loc_new[1], label='transform pose',c='r', marker="v")
plt.scatter(U_new[0,:], U_new[1,:], label='transform landmarks',c='r')

plt.axis('equal')
plt.title('residuals btw scan and ref', fontsize=15)
plt.legend()
plt.show()
