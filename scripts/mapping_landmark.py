'''find trunk(landmark) GPS in a pgm/jpg map'''

'''math tool'''
import csv
import numpy as np
from scipy.spatial import distance as dist

'''plot tool'''
import matplotlib.pyplot as plt

'''image tool'''
import cv2
import statistics as sta

'''gps'''
import utm
from pyproj import Proj
import shapefile

import os
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")

# directory = '/home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/NTU_allMaps/'
# bag_name = '210906_loopClosure/' #'ntu_test3_2021-07-25-18-23-39/'
# directory = '/home/ncslaber/110-1/211002_allLibrary/'
# bag_name = '2021-10-03-17-48-23_back-left-2/'
file_path = '/home/ncslaber/110-1/211002_allLibrary/2021-10-02-17-54-09/1116_test2_after_preprocess/' #directory+bag_name
shp_path = file_path + 'shapefiles/'
os.makedirs(os.path.dirname(file_path), exist_ok=True)
os.makedirs(os.path.dirname(shp_path), exist_ok=True)

resolution_pixel=0.05

def get_init_utm():
    with open(file_path+'lat_lon.csv', 'r') as csvfile:
        lat, lng = csv.reader(csvfile, delimiter=',')
    lat = float(lat[0])
    lng = float(lng[0])
    _, _, zone, R = utm.from_latlon(lat, lng)
    
    return lng, lat, zone, R

def morph_map(raw_pgm, raw_pgm_binary):
    kernel = np.ones((3,3), np.uint8)
    raw_pgm_binary = cv2.erode(raw_pgm_binary, kernel, iterations = 1)
    raw_pgm_binary = cv2.dilate(raw_pgm_binary,kernel,iterations = 1)
    fig = plt.figure(figsize=(10,10))
    subplot = fig.add_subplot(121)
    subplot.imshow(cv2.cvtColor(raw_pgm, cv2.COLOR_BGR2RGB))
    subplot = fig.add_subplot(122)
    subplot.imshow(cv2.cvtColor(raw_pgm_binary, cv2.COLOR_BGR2RGB))
    plt.show()
    # cv2.imwrite(file_path+'raw_pgm_binary.png', raw_pgm_binary)

def filter_labels(num_objects, labels):
    filter_labels_list = []
    for i in range(num_objects):
        if len(labels[labels==i])>10 and len(labels[labels==i])<1000:
            filter_labels_list.append(i)
    return filter_labels_list

def get_matched_circle(raw_pgm_binary, filter_labels_list):
    centroid_rawList = []
    circle_bd = np.zeros(raw_pgm_binary.shape, dtype=np.uint8)
    for i in filter_labels_list:
        A = []
        for x in range(raw_pgm_binary.shape[0]):
            for y in range(raw_pgm_binary.shape[1]):
                # if labels[x][y] == 23 or labels[x][y] == 41:
                #     A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
                if labels[x][y] == i:
                    A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
        A = np.asarray(A)
        print('new one circle # of points: ', A.shape, i)
        
        k = np.linalg.inv(A.T @ A)
        k = k @ A.T
        k = k @ np.ones((k.shape[1],1))
        
        centre_x = k[0][0]/(-2)
        centre_y = k[1][0]/(-2)
        radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])
        print('x,y,r: ', int(centre_x+0.5), int(centre_y+0.5), int(radius_r+0.5))
        
        centroid_rawList.append((int(centre_x+0.5),int(centre_y+0.5),int(radius_r+0.5)))

        cv2.circle(circle_bd,(int(centre_y), int(centre_x)), int(radius_r+0.5), 150, 2)
        # cv2.putText(circle_bd, #numpy array on which text is written
        #             str(int(centre_x))+','+str(int(centre_y))+','+str(int(radius_r)), #text
        #             (int(centre_y)-20,int(centre_x)-20), #position at which writing has to start
        #             cv2.FONT_HERSHEY_SIMPLEX, #font family
        #             0.6, #font size
        #             255, #font color
        #             1, cv2.LINE_AA) #font stroke
        cv2.imshow('circle_bd',circle_bd)
        cv2.waitKey(100)
    circle_bd[raw_pgm_binary==255]=255
    fig1,ax1 = plt.subplots(figsize=(8,8))
    plt.imshow(cv2.cvtColor(circle_bd, cv2.COLOR_BGR2RGB))
    cv2.imwrite(file_path+'raw_circle_bd.png', circle_bd)

    return centroid_rawList

def get_merging_circle(raw_pgm_binary, filter_labels_list):
    circle_bd = np.zeros(raw_pgm_binary.shape, dtype=np.uint8)
    centroid_filteredList = []

    while len(centroid_rawList) > 0:
        count = 1
        centroid = centroid_rawList.pop(0)
        
        indexX = centroid[0]
        indexY = centroid[1]
        indexR = centroid[2]
        i = 0 
        # leng = len(centroidList)-1
        while True:
            if i < len(centroid_rawList):
                eDist = dist.euclidean(np.asarray(centroid), np.asarray(centroid_rawList[i]))
                if eDist < 25: # 1m
                    print("merging centroid...")
                    centroid_tmp = centroid_rawList.pop(i)
                    indexX += centroid_tmp[0]
                    indexY += centroid_tmp[1]
                    indexR += centroid_tmp[2]
                    print('     x,y,r: ', centroid_tmp)
                    count += 1
                    centroid = (indexX/count, indexY/count, indexR/count)
                    i -= 1
                i += 1
            else:
                break
        centroid_filteredList.append(centroid)
        cv2.circle(circle_bd, (int(centroid[1]),int(centroid[0])), int(centroid[2]), 150, 3) # radius is 8 pixel
        # cv2.putText(circle_bd, #numpy array on which text is written
        #             str(int(centroid[0]))+','+str(int(centroid[1]))+','+str(int(centroid[2])), #text
        #             (int(centroid[1])-20,int(centroid[0])-20), #position at which writing has to start
        #             cv2.FONT_HERSHEY_SIMPLEX, #font family
        #             0.6, #font size
        #             255, #font color
        #             1, cv2.LINE_AA) #font stroke
    circle_bd[raw_pgm_binary==255]=255
    fig2,ax2 = plt.subplots(figsize=(15,15))
    plt.imshow(cv2.cvtColor(circle_bd, cv2.COLOR_BGR2RGB))
    tmp = np.zeros((circle_bd.shape[0],circle_bd.shape[1],3))
    
    tmp[circle_bd==0,:]=(255,255,255)
    tmp[circle_bd==255,:]=(100,100,100)
    tmp[circle_bd==150,:]=(0,0,255)
    cv2.imwrite(file_path+'filtered_circle_bd_color.png', tmp)

    np.save(file_path+'centroid_filteredList', centroid_filteredList)

    return centroid_filteredList 

def transform_from_pixel2m(cX, cY,length):  # Here effects initial position
    # cX_m, cY_m = transformation(cX, cY, -0.5*np.pi, -int(length*(1-map_start_y)), int(length*(1-map_start_x)))
    
    cX_m = cX - int(length*0.5) # initial position in the map #0.3
    cY_m = cY - int(length*0.5) # 0.5
    
    cX_m *= resolution_pixel
    cY_m *= resolution_pixel
    return cX_m, cY_m

def get_utm_negBDs_from_center(index, cX_m, cY_m, cR_m, utm_x_ref, utm_y_ref):
    number_of_point = 12
    piece_rad = np.pi/(number_of_point/2)
    neg_bd = []
    for i in range(number_of_point):
        neg_bd.append((cX_m+(cR_m+0.3)*np.cos(piece_rad*i)+utm_x_ref, cY_m+(cR_m+0.3)*np.sin(piece_rad*i)+utm_y_ref))
    neg_bd = np.asarray(neg_bd)
    plt.scatter(neg_bd[:,0], neg_bd[:,1], c='b', s=10)
    plt.scatter(cX_m+utm_x_ref, cY_m+utm_y_ref, c='g')
    np.save(shp_path+'neg_'+str(index+1)+'_bd_utm', neg_bd)
    np.save(shp_path+'center_'+str(index+1)+'_bd_utm', (cX_m+utm_x_ref, cY_m+utm_y_ref, cR_m))

    return neg_bd

def save_shp(index, neg_bd):
    gps_lat, gps_lon = utm.to_latlon(neg_bd[:,0], neg_bd[:,1], zone, R)
    gps_latlon = np.vstack(( gps_lat, gps_lon))
    gps_latlon = np.transpose(gps_latlon)
    w = shapefile.Writer(shp_path+'neg_'+str(j+1))
    w.field('LAT_LON', 'C', '40')
    w.multipoint(gps_latlon) 
    w.record('neg'+str(index+1))
    w.close()
    print("save one shp")

def draw_click_circle(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(raw_pgm, (x+300, y+500), 6, 255, -1)

def reduce_noise(raw_pgm):
    print("manually reduce noise---")
    cv2.namedWindow('raw_pgm')
    cv2.setMouseCallback('raw_pgm', draw_click_circle)
    while True:
        cv2.imshow('raw_pgm', raw_pgm)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("break click!")
            break
    cv2.imwrite(file_path+'raw_modified.png', raw_pgm)

if __name__=="__main__":
    ''' read map '''
    # file_path = '/home/ncslaber/110-1/211002_allLibrary/2021-10-02-17-29-15/1116_test2_after_preprocess/'
    raw_pgm = cv2.imread(file_path+"1118.pgm")
    # raw_pgm = cv2.imread(file_path+"raw_modified.png")
    if raw_pgm is None:
        print("Image is empty!!")
    raw_pgm = cv2.cvtColor(raw_pgm, cv2.COLOR_RGB2GRAY)
    (width, height) = raw_pgm.shape # the order is right
    print("pgm height is: ",height)
    # raw_pgm = cv2.resize(raw_pgm, (1024, 1024), interpolation=cv2.INTER_AREA)
    cv2.imshow("raw_pgm",cv2.resize(raw_pgm, (700, 700)))
    cv2.waitKey(100)
    
    ''' reduce noise '''   
    # reduce_noise(raw_pgm[500:1300,300:1600])#
    

    ''' preprocess the map '''
    raw_pgm_binary = np.zeros(raw_pgm.shape[:2],dtype=np.uint8)
    raw_pgm_binary[raw_pgm==0]=255
    morph_map(raw_pgm, raw_pgm_binary)

    ''' find connected component and circle matching'''
    num_objects, labels = cv2.connectedComponents(raw_pgm_binary)
    print('>>>># of raw objects:',num_objects)

    filter_labels_list = filter_labels(num_objects, labels)
    print('>>>># of size-filtered  objects:',len(filter_labels_list))
    # np.save('/home/ncslaber/filter_labels_list', filter_labels_list)

    centroid_rawList = get_matched_circle(raw_pgm_binary, filter_labels_list)
    
    '''merge close trunk by repeatedly move average'''
    centroid_filteredList = get_merging_circle(raw_pgm_binary, filter_labels_list)
    # centroid_filteredList = np.load(file_path+'centroid_filteredList.npy')
    print('>>>># of dist-filtered  objects:',len(centroid_filteredList))
    centroid_filteredList = np.asarray(centroid_filteredList)
    centre_x_list = centroid_filteredList[:,0]
    centre_y_list = centroid_filteredList[:,1]
    radius_list = centroid_filteredList[:,2]

    ''' find neg bds of trunk '''
    # lng, lat, zone, R = get_init_utm()
    # proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    # utm_x_ref, utm_y_ref = proj(lng, lat)
    utm_x_ref, utm_y_ref = 352878.8862176978, 2767708.6000474878
    
    fig2,ax2 = plt.subplots(figsize=(7,5))
    for j in range(centre_x_list.shape[0]):
        cX = centre_x_list[j]
        cY = centre_y_list[j]
        cX_m, cY_m = transform_from_pixel2m(cX, cY, raw_pgm.shape[0])
        cR_m = radius_list[j]/20
        
        neg_bd = get_utm_negBDs_from_center(j, cX_m, cY_m, cR_m, utm_x_ref, utm_y_ref)
        
        '''each negs save one shp'''
        # save_shp(j,neg_bd)

    plt.ylabel('UTM Y [m]', fontsize=22)
    plt.xticks(fontsize=18 )
    plt.xlabel('UTM X [m]', fontsize=22)
    plt.yticks(fontsize=18 )
    plt.grid('on')
    plt.axis('equal')
    plt.title('neg bds of trunks', fontsize=15)
    plt.draw()

    '''draw with pos'''
    fig3, ax3 = plt.subplots(figsize=(8, 8))
    plt.grid(True)
    plt.axis('equal')
    for i in range(len(centre_x_list)):
        neg_bd = np.load(shp_path+'neg_'+str(i+1)+'_bd_utm.npy')
        plt.scatter(neg_bd[:,0], neg_bd[:,1], c='b')

    # r = shapefile.Reader('/home/ncslaber/shapefiles/test/pos/pos_Dahu_enlarge')
    # pos_bd = r.shape(0).points
    # pos_bd = np.asarray(pos_bd)
    # ux, uy = proj(pos_bd[:,1], pos_bd[:,0])
    # plt.plot(ux,uy,'-o',c='black',label='preset positive bd')
    # ax3.get_xaxis().get_major_formatter().set_useOffset(
    #         round(min(ux) / 1000, 2)*1000)
    plt.ylabel('UTM Y [m]', fontsize=22)
    plt.xticks(fontsize=18 )
    plt.xlabel('UTM X [m]', fontsize=22)
    plt.yticks(fontsize=18 )
    plt.axis('equal')
    plt.show()