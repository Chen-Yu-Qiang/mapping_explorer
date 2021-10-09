import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.ticker import NullFormatter, FixedLocator
from matplotlib import transforms
from pyproj import Proj
import utm

bag_source_file = "/home/ncslaber/110-1/211002_allLibrary/image_bag/2021-10-02-17-54-09.bag"
posBDs_source_file = "/home/ncslaber/109-2/210816_NTU_half/positiveBDs_2021-08-15-11-23-22.bag"

shp_path = '/home/ncslaber/110-1/211002_allLibrary/2021-10-02-17-54-09/shapefiles/'

topics = ['/navsat/fix']#'/navsat/fix'

try:
    ''' Read exploring bag '''
    print('Start reading: {}'.format(bag_source_file))

    lat = []
    lng = []

    bag = rosbag.Bag(bag_source_file, 'r')
    for topic, msg, t in bag.read_messages(topics=topics):
        
        if topic == '/navsat/fix':
            if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                lat.append(msg.latitude)
                lng.append(msg.longitude)
            else:
                print("error lat/lon:", msg.latitude, msg.longitude)
    bag.close()

    lat = np.asarray(lat)
    lng = np.asarray(lng)
    gps_xy = np.vstack(( lng, lat))
    gps_xy = np.transpose(gps_xy)
    # np.save('/home/ncslaber/traj_GPS', gps_xy)

    # plot
    fig, ax = plt.subplots(figsize=(12, 12), dpi=100)
    plt.grid(True)
    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(0)#
    
    _, _, zone, _ = utm.from_latlon(lat[0], lng[0])
    print('lat: ', lat.shape)
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    ux, uy = proj(lng, lat)

    print(np.asarray(ux).shape)
    colors = cm.rainbow(np.linspace(1, 0, np.asarray(ux[:5000]).shape[0]))
    plt.scatter(ux[:5000], uy[:5000], c=colors, transform = rot + base, label="exploring")
    utm_xy = np.vstack(( ux[:5000], uy[:5000]))
    # np.save(bag_source_file[:-4]+'/traj_GPS_filtered', utm_xy)
    ax.axis('equal')
    plt.title("UTM [m]", fontsize=25)

    plt.yticks(fontsize=15)
    plt.xticks(fontsize=15)

    for i in range(8):
        neg_bd = np.load(shp_path+'neg_'+str(i+1)+'_bd_utm.npy')
        center = np.load(shp_path+'center_'+str(i+1)+'_bd_utm.npy')
        if neg_bd is None or center is None:
            print("neg_bd is empty!!")
        plt.scatter(neg_bd[:,0], neg_bd[:,1], c='b', s=10)
        plt.scatter(center[0], center[1], c='b', marker='X',s=100)

    ax.ticklabel_format(useOffset=False, style='sci')
    plt.show()

    ''' Read pos bds bag 
    print('Start reading: {}'.format(posBDs_source_file))
    lat = []
    lng = []

    bag = rosbag.Bag(posBDs_source_file, 'r')
    for topic, msg, t in bag.read_messages(topics=topics):
        
        if topic == '/navsat/fix':
            if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                lat.append(msg.latitude)
                lng.append(msg.longitude)
            else:
                print("error lat/lon:", msg.latitude, msg.longitude)
    bag.close()
    print(lat[0],lng[0])
    lat = np.asarray(lat)
    lng = np.asarray(lng)
    gps_xy = np.vstack(( lat, lng))
    gps_xy = np.transpose(gps_xy)

    # plot
    plt.grid(True)
    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(-40)#
    
    _, _, zone, _ = utm.from_latlon(lat[0], lng[0])
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    ux, uy = proj(lng, lat)

    print(np.asarray(ux).shape)
    # colors = cm.rainbow(np.linspace(1, 0, np.asarray(ux).shape[0]))
    plt.scatter(ux, uy, c='k', transform = rot + base, label="pos_bds")
    utm_xy = np.vstack(( ux, uy))
    # utm_xy = np.transpose(utm_xy)
    np.save('/home/ncslaber/pos_traj_GPS', utm_xy)

    ax.axis('equal')
    plt.title("UTM [m]", fontsize=25)

    plt.yticks(fontsize=15)
    plt.xticks(fontsize=15)

    ax.ticklabel_format(useOffset=False, style='sci')
    plt.legend(fontsize=25)
    #plt.savefig(path+"test_timeMarker.png", dpi=100)
    plt.show()
    '''

except:
    print('error with: {}'.format(bag_source_file)) 