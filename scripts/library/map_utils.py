'''math tool'''
import csv
import numpy as np
from scipy.spatial import distance as dist

'''plot tool'''
import matplotlib.pyplot as plt
from matplotlib import transforms

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

bag_files = ['2021-10-02-17-03-14', '2021-10-02-17-54-09', '2021-10-02-17-29-15', '2021-10-03-17-48-23', '2021-07-25-18-23-39']
file_path = '/home/ncslaber/110-1/211002_allLibrary/'

center = [(352869, 2767701), (352878, 2767709), (352875, 2767711), (352897, 2767682), (352866,2767646)]
npcenter = np.asarray(center)
# plt.scatter(npcenter[:,0], npcenter[:,1], s=50)
# plt.title('start position of recorded dataset', fontsize=20)
# plt.yticks(fontsize=15)
# plt.xticks(fontsize=15)
# plt.grid(True)
# plt.axis('equal')
# plt.show()

resolution_pixel=0.05

def transform_from_pixel2m(cX, cY,length):  # Here effects initial position
    # cX_m, cY_m = transformation(cX, cY, -0.5*np.pi, -int(length*(1-map_start_y)), int(length*(1-map_start_x)))
    
    cX_m = cX - int(length*0.5) # initial position in the map #0.3
    cY_m = cY - int(length*0.5) # 0.5
    
    cX_m *= resolution_pixel
    cY_m *= resolution_pixel
    return cX_m, cY_m

if __name__=="__main__":
    # raw_pgm = cv2.imread(file_path+"back-left-2.pgm")
    raw_pgm_list = []
    for bag_file in bag_files:
        raw_pgm = (cv2.imread(file_path+bag_file+"/raw_modified.png"))
        if raw_pgm is None:
            print("Image is empty!!")
        raw_pgm = cv2.cvtColor(raw_pgm, cv2.COLOR_RGB2GRAY)
        (width, height) = raw_pgm.shape # the order is right
        print("pgm height is: ",height)

        cv2.imshow("raw_pgm",cv2.resize(raw_pgm, (700, 700)))
        cv2.waitKey(200)
        # raw_pgm = cv2.resize(raw_pgm, (102, 102))
        temp = np.zeros(raw_pgm.shape[:2],dtype=np.uint8)
        temp[raw_pgm==0] = 255
        raw_pgm_list.append(temp)

    # raw_pgm = cv2.resize(raw_pgm, (102, 102))
    fig, ax = plt.subplots() #figsize=(200,200),dpi=120

    canvas = np.zeros((2048+65*20,2048+31*20), dtype='uint8')
    canvas[2048+65*20-1024-512:2048+65*20-512,512:1024+512] = raw_pgm_list[4]
    canvas[2048+65*20-36*20-2048:2048+65*20-36*20, 2048+31*20-2048:2048+31*20] = np.bitwise_or(raw_pgm_list[3], canvas[2048+65*20-36*20-2048:2048+65*20-36*20, 2048+31*20-2048:2048+31*20])
    canvas[10*20:10*20+2048,3*20:3*20+2048] = np.bitwise_or(raw_pgm_list[0], canvas[10*20:10*20+2048,3*20:3*20+2048])
    canvas[3*20:3*20+2048,12*20:12*20+2048] = np.bitwise_or(raw_pgm_list[1], canvas[3*20:3*20+2048,12*20:12*20+2048])
    canvas[:2048,9*20:9*20+2048] = np.bitwise_or(raw_pgm_list[2], canvas[:2048,9*20:9*20+2048])
    
    base = plt.gca().transData
    translate = transforms.Affine2D().translate(0, 0)
    ax.set_aspect('equal')
    world_bounds = [0,205]
    
    plt.imshow(canvas[400:2500, 500:2000], transform=base+translate)
    plt.title('NTU library', fontsize=25)
    plt.yticks(fontsize=20)
    plt.xticks(fontsize=20)
    # plt.legend(fontsize=15)
    plt.show()

    # cv2.imwrite('1007-map.png',canvas[400:2500, 500:2000])