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

file_path = "/home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/0906_demo_data/10/"

with open(file_path+'lat_lon.csv', 'r') as csvfile:
    lat, lng = csv.reader(csvfile, delimiter=',')
lat = float(lat[0])
lng = float(lng[0])
_, _, zone, R = utm.from_latlon(lat, lng)
proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
utm_x_ref, utm_y_ref = proj(lng, lat)

''' load local map '''
raw_pgm = cv2.imread("/home/ncslaber/1.pgm")
# print(raw_pgm.shape[:2])
raw_pgm_binary = np.zeros(raw_pgm.shape[:2],dtype=np.uint8)
raw_pgm_binary[raw_pgm[:,:,0]==0]=255
fig = plt.figure(figsize=(10,10))
subplot = fig.add_subplot(121)
subplot.imshow(cv2.cvtColor(raw_pgm, cv2.COLOR_BGR2RGB))
subplot = fig.add_subplot(122)
subplot.imshow(cv2.cvtColor(raw_pgm_binary, cv2.COLOR_BGR2RGB))


''' find connected component and push into point array A '''
num_objects, labels = cv2.connectedComponents(raw_pgm_binary)
centre_x_list = []
centre_y_list = []
trunk_radius_utm_list = []
centroid_rawList = []
circle_bd = np.zeros(raw_pgm_binary.shape, dtype=np.uint8)
print('>>>>num_objects:',num_objects)

for i in range(num_objects-1):
    A = []
    for x in range(1024):
        for y in range(1024):
            if labels[x][y] == i+1:
                A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
    A = np.asarray(A)
    print('# of points: ',A.shape)
    
    if A.shape[0] < 10:
        continue

    k = np.linalg.inv(A.T @ A)
    k = k @ A.T
    k = k @ np.ones((k.shape[1],1))
    # print(k)

    centre_x = k[0][0]/(-2)
    centre_y = k[1][0]/(-2)
    radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])
    print('x,y,r: ', int(centre_x+0.5), int(centre_y+0.5), int(radius_r+0.5))
    
    centroid_rawList.append((int(centre_x+0.5),int(centre_y+0.5)))
    centre_x_list.append(int(centre_x+0.5))
    centre_y_list.append(int(centre_y+0.5))
    trunk_radius_utm_list.append(int(radius_r+0.5))

    cv2.circle(circle_bd,(int(centre_y), int(centre_x)), int(radius_r+0.5), 150, 2)
    cv2.putText(circle_bd, #numpy array on which text is written
                str(int(centre_x))+','+str(int(centre_y)), #text
                (int(centre_y)-20,int(centre_x)-20), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.2, #font size
                255, #font color
                1, cv2.LINE_AA) #font stroke
circle_bd[raw_pgm_binary==255]=255

fig1,ax1 = plt.subplots(figsize=(8,8))
# plt.xlim((500,700))
# plt.ylim((250,50))
plt.title('current found trunk', fontsize=15)
plt.imshow(cv2.cvtColor(circle_bd, cv2.COLOR_BGR2RGB))

def transform_from_pixel2m(cX, cY,length):  # Here effects initial position
    # cX_m, cY_m = transformation(cX, cY, -0.5*np.pi, -int(length*(1-map_start_y)), int(length*(1-map_start_x)))
    
    cX_m = cX - int(length*(1-0.5))
    cY_m = cY - int(length*(1-0.5))
    
    cX_m /= 20
    cY_m /= 20
    return cX_m, cY_m

''' find neg bds of trunk '''
fig2 = plt.figure(figsize=(10,10))
centre_x_list = np.asarray(centre_x_list)
centre_y_list = np.asarray(centre_y_list)
trunk_radius_utm_list = np.asarray(trunk_radius_utm_list)

''' load robot current pose '''
with open(file_path+'cb_pose.csv', 'r') as csvfile:
    imu_yaw, lat, lng = csv.reader(csvfile, delimiter=',')
imu_yaw = float(imu_yaw[0])
lat = float(lat[0])
lng = float(lng[0])
_, _, zone, R = utm.from_latlon(lat, lng)
proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
utm_x_loc_origin, utm_y_loc_origin = proj(lng, lat)

cX_utm_loc = []
cY_utm_loc = []
print(len(centre_x_list))
for j in range(centre_x_list.shape[0]):
    cX = centre_x_list[j]
    cY = centre_y_list[j]
    print(cX, cY)
    trunk_radius_utm = trunk_radius_utm_list[j]/10
    cX_m, cY_m = transform_from_pixel2m(cX, cY, raw_pgm.shape[0])
    print(cX_m, cY_m)

    cX_utm_loc.append(cX_m+utm_x_ref+1)
    cY_utm_loc.append(cY_m+utm_y_ref)

    plt.scatter(cX_m+utm_x_ref, cY_m+utm_y_ref, c='b')

cX_utm_loc = np.asarray(cX_utm_loc)
cX_utm_loc = np.asarray(cX_utm_loc)
center_utm_loc = np.vstack((cX_utm_loc,cY_utm_loc))
print(center_utm_loc)

''' load landmark map '''
directory = '/home/ncslaber/109-2/210725_NTU_leftAreaLibrary/'
bag_name = 'ntu_test3_2021-07-25-18-23-39/'
file_path_map = directory+bag_name
center_utm_ref = np.load(file_path_map+'center_utm_ref.npy')

cX_utm_ref = center_utm_ref[0,:]
cY_utm_ref = center_utm_ref[1,:]

plt.scatter(cX_utm_ref, cY_utm_ref, c='g', label='reference landmarks', marker='X',s=100)
# plt.scatter(utm_x_ref, utm_y_ref, label='start_recording',c='black')
plt.axis('equal')
plt.title('global map in UTM', fontsize=15)
plt.legend()
plt.draw()


''' find rigid transformation '''
P = center_utm_ref
# U = center_utm_loc

U = np.array([[ 352846.31057329,  352849.9848004 ],
       [2767649.02233138, 2767649.94567053]])
utm_loc = np.array([[utm_x_loc_origin],[utm_y_loc_origin]])
resid_scalar = 50

def find_second(row_index, first_min):
    second_min = 100
    sm_index = 0
    row_list = D[row_index,:]
    for i in range(len(row_list)):
        if row_list[i] != first_min:
            if row_list[i] < second_min:
                second_min = row_list[i]
                sm_index = i
    
    return second_min, sm_index
count = 0
while resid_scalar > 1:
    count += 1
    D = dist.cdist(U.T, P.T)
    print('>>>> start to ICP >>>>')
    print('dist: ', D)
    # rows = D.min(axis=1)
    cols = D.argmin(axis=1)
    print('match point: ', list(enumerate(cols)))

    for a in range(len(cols)):
        b=1
        while a+b < len(cols):
            if cols[a] == cols[a+b]:
                print("matched same ref landmarks!")
                second_min_A,A = find_second(a,D[a,cols[a]])
                second_min_B,B = find_second(a+b,D[a+b,cols[a+b]])
                if second_min_A>second_min_B:
                    cols[a+b]=B
                elif second_min_A<second_min_B:
                    cols[a]=A
                print('changed point: ', list(enumerate(cols)))
            b+=1
        
    Q = np.zeros(U.shape)
    for (row, col) in enumerate(cols):
        Q[:,row] = P[:,col]
    print(Q)
    print(U)
    U_bar = np.array([np.average(U, axis=1)])
    U_bar = U_bar.T
    Q_bar = np.array([np.average(Q, axis=1)])
    Q_bar = Q_bar.T

    X = U-U_bar
    Y = Q-Q_bar
    S = X @ Y.T
    u, s, vh = np.linalg.svd(S)
    # check u, vh det
    print('det(vh@u.T): ',np.linalg.det(vh@u.T))
    det = np.linalg.det(vh@u.T)
    if det>0:
        tmp = np.array([[1,0],[0,1]])
    else: 
        tmp = np.array([[1,0],[0,-1]])
    R = vh @ tmp @ u.T
    t = Q_bar-U_bar

    print('R: ',R)
    print('theta (deg): ',(np.arctan2(-R[1,0], R[0,0]))/np.pi*180)
    print('translation: ',t)
    U_new = R@X+U_bar+t
    utm_loc_decentral = utm_loc-U_bar
    utm_loc_new = R@utm_loc_decentral+U_bar+t

    # calculate residuals
    residuals = Q-U_new
    residuals = np.absolute(residuals)
    resid_scalar = residuals.sum()
    print("residual = ",resid_scalar)
    U = U_new
    utm_loc = np.array([[utm_loc_new[0][0]],[utm_loc_new[1][0]]])
    print("iteration time: ", count)
    if count>4:
        print("iterate over 5 times!!")
        break

''' plot result '''
traj = np.load('/home/ncslaber/109-2/210725_NTU_leftAreaLibrary/ntu_test2_2021-07-25-17-42-59/traj_GPS_filtered.npy')
traj_x = traj[0,:]
traj_y = traj[1,:]
fig5 = plt.figure(figsize=(15,15))
plt.scatter(cX_utm_ref, cY_utm_ref, c='g', label='ref landmarks', marker='X',s=700)
# plt.scatter(utm_x_ref, utm_y_ref, label='start_recording',c='black')
plt.scatter(cX_utm_loc, cY_utm_loc, label='scanned landmarks',c='b',s=300)
plt.scatter(utm_x_loc_origin, utm_y_loc_origin, label='initial robot pose',c='b', marker="v",s=300)
plt.scatter(traj_x[0:len(traj_x)-300:100], traj_y[0:len(traj_y)-300:100],c='black',s=20)

plt.scatter(U_new[0,:], U_new[1,:], label='transform landmarks',c='r',s=300)
plt.scatter(utm_loc_new[0], utm_loc_new[1], label='transform pose',c='r', marker="v",s=300)

plt.axis('equal')
plt.title('residuals btw scan and ref', fontsize=30)
plt.legend(fontsize=30)
plt.show()


