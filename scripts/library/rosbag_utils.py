import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.ticker import NullFormatter, FixedLocator
from matplotlib import transforms
from pyproj import Proj
import utm

def initial_UTM(duration_s=3):
    bag_source_file = "/home/ncslaber/110-1/211002_allLibrary/image_bag/2021-10-03-17-48-23.bag"

    topic = ['/navsat/fix']

    duration_n = duration_s*5
    cn = 0

    try:
        ''' Read exploring bag '''
        print('Start reading: {}'.format(bag_source_file))

        lat = []
        lng = []

        bag = rosbag.Bag(bag_source_file, 'r')
        for topic, msg, t in bag.read_messages(topics=topic):
            if cn > duration_n: 
                break
            if topic == '/navsat/fix':
                if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                    lat.append(msg.latitude)
                    lng.append(msg.longitude)
                else:
                    print("error lat/lon:", msg.latitude, msg.longitude)
            cn += 1
        bag.close()

    except:
        print('error with: {}'.format(bag_source_file)) 

    lat = np.average( np.asarray(lat) )
    lng = np.average( np.asarray(lng) )
    print(lng,lat)
    _, _, zone, _ = utm.from_latlon(lat, lng)
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    ux, uy = proj(lng, lat)

    return ux, uy

if __name__ == '__main__':
    print(initial_UTM(duration_s=3))