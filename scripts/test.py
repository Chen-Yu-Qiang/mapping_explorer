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
import shapefile

import os
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")

directory = '/home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/0906_demo_data/demo/'
bag_name = ''#'ntu_test3_2021-07-25-18-23-39/'
file_path = directory+bag_name
shp_path = file_path + 'shapefiles/'
os.makedirs(os.path.dirname(file_path), exist_ok=True)
os.makedirs(os.path.dirname(shp_path), exist_ok=True)

with open(file_path+'lat_lon.csv', 'r') as csvfile:
    lat, lng = csv.reader(csvfile, delimiter=',')
lat = float(lat[0])
lng = float(lng[0])

_, _, zone, R = utm.from_latlon(lat, lng)
proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
utm_x_ref, utm_y_ref = proj(lng, lat)

''' read map '''
raw_pgm = cv2.imread(file_path+"demo.pgm")
raw_pgm = cv2.cvtColor(raw_pgm, cv2.COLOR_RGB2GRAY)
(width, height) = raw_pgm.shape # the order is right
print("pgm height is: ",height)
cv2.imshow("raw_pgm",cv2.resize(raw_pgm, (700, 700)))
cv2.waitKey(100)
cv2.imwrite(file_path+'raw_modified.png', raw_pgm)

def draw_click_circle(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(raw_pgm, (x, y), 20, 255, -1)

print("manually reduce noise---")
''' reduce noise '''
cv2.namedWindow('raw_pgm')
cv2.setMouseCallback('raw_pgm', draw_click_circle)
while True:
    cv2.imshow('raw_pgm', raw_pgm)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("break click!")
        break

''' preprocess the map '''
raw_pgm_binary = np.zeros(raw_pgm.shape[:2],dtype=np.uint8)
raw_pgm_binary[raw_pgm==0]=255
kernel = np.ones((3,3), np.uint8)
raw_pgm_binary = cv2.dilate(raw_pgm_binary,kernel,iterations = 1)
raw_pgm_binary = cv2.erode(raw_pgm_binary, kernel, iterations = 1)
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
plt.imshow(cv2.cvtColor(circle_bd, cv2.COLOR_BGR2RGB))

def transform_from_pixel2m(cX, cY,length):  # Here effects initial position
    # cX_m, cY_m = transformation(cX, cY, -0.5*np.pi, -int(length*(1-map_start_y)), int(length*(1-map_start_x)))
    
    cX_m = cX - int(length*(1-0.5))
    cY_m = cY - int(length*(1-0.5))
    
    cX_m /= 20
    cY_m /= 20
    return cX_m, cY_m

''' find neg bds of trunk '''
fig2,ax2 = plt.subplots(figsize=(7,5))
centre_x_list = np.asarray(centre_x_list)
centre_y_list = np.asarray(centre_y_list)
trunk_radius_utm_list = np.asarray(trunk_radius_utm_list)

number_of_point = 12
piece_rad = np.pi/(number_of_point/2)
print(len(centre_x_list))
for j in range(centre_x_list.shape[0]):
    neg_bd = []
    cX = centre_x_list[j]
    cY = centre_y_list[j]
    # print(cX, cY)
    trunk_radius_utm = trunk_radius_utm_list[j]/10
    cX_m, cY_m = transform_from_pixel2m(cX, cY, raw_pgm.shape[0])
    # print(cX_m, cY_m)

    for i in range(number_of_point):
        neg_bd.append((cX_m+trunk_radius_utm*np.cos(piece_rad*i)+utm_x_ref, cY_m+trunk_radius_utm*np.sin(piece_rad*i)+utm_y_ref))
    neg_bd = np.asarray(neg_bd)
    plt.scatter(neg_bd[:,0], neg_bd[:,1], c='b', s=10)
    plt.scatter(cX_m+utm_x_ref, cY_m+utm_y_ref, c='g')
    np.save(shp_path+'neg_'+str(j+1)+'_bd_utm', neg_bd)

    gps_lat, gps_lon = utm.to_latlon(neg_bd[:,0], neg_bd[:,1], zone, R)
    gps_latlon = np.vstack(( gps_lat, gps_lon))
    gps_latlon = np.transpose(gps_latlon)
    w = shapefile.Writer(shp_path+'neg_'+str(j+1))
    w.field('LAT_LON', 'C', '40')
    w.multipoint(gps_latlon) 
    w.record('neg'+str(j+1))
    w.close()

plt.ylabel('UTM Y [m]', fontsize=22)
plt.xticks(fontsize=18 )
plt.xlabel('UTM X [m]', fontsize=22)
plt.yticks(fontsize=18 )
plt.grid('on')
plt.axis('equal')
plt.title('neg bds of trunks', fontsize=15)
plt.draw()

fig3, ax3 = plt.subplots(figsize=(8, 8))
plt.grid(True)
plt.axis('equal')
for i in range(len(centre_x_list)):
    neg_bd = np.load(shp_path+'neg_'+str(i+1)+'_bd_utm.npy')
    plt.scatter(neg_bd[:,0], neg_bd[:,1], c='b')

r = shapefile.Reader('/home/ncslaber/shapefiles/test/pos/pos_NTU_left')
pos_bd = r.shape(0).points
pos_bd = np.asarray(pos_bd)
ux, uy = proj(pos_bd[:,1], pos_bd[:,0])
plt.plot(ux,uy,'-o',c='black',label='preset positive bd')
ax3.get_xaxis().get_major_formatter().set_useOffset(
        round(min(ux) / 1000, 2)*1000)
plt.ylabel('UTM Y [m]', fontsize=22)
plt.xticks(fontsize=18 )
plt.xlabel('UTM X [m]', fontsize=22)
plt.yticks(fontsize=18 )
plt.axis('equal')
plt.show()