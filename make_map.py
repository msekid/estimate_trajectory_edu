import numpy as np
import overpy
import folium
import statistics
import math
from init import Init
import gps2coord, coord2gps
from scipy import interpolate
#from temp import ExtractRoads
from optimize_gps import OptimizeGPS
from optimize_traj7 import OptimizeTraj
from explore_gps import ExploreGPS
gps_t = Init().gps_t
gps_list = []
coord_map = np.loadtxt('extracted_roads.csv', encoding="shift-jis", delimiter=',')
m = folium.Map(location=gps_t[0], zoom_start=20)
for i in range(len(coord_map)):
    gps = coord2gps.calc_lat_lon(np.array(coord_map[i][0]), np.array(coord_map[i][1]), gps_t[0][0], gps_t[0][1])
    gps_list.append([gps[0], gps[1]])
    folium.Circle(gps_list[i], radius=1, color='gray', fill=False).add_to(m)
#folium.PolyLine(gps_list, color='black', weight=1.0).add_to(m)  
#latlon_ = ExtractRoads(extract_dist=300).make_latlon_frommap()
optimized = OptimizeTraj().calcOptimizeTraj()
exp = ExploreGPS().explore_gps(threshold=1.5)
exp_ = ExploreGPS()
coord_selected_ = exp[1]
coord_selected = exp[2]
#gps_t_ex = np.loadtxt('exploreGPS.csv', encoding="shift_jisx0213", delimiter=',')


est_i0 = np.load('i0_list.npy')
est_i10 = np.load('i10_list.npy')
est_i10_explore = np.load('i10_explore_list.npy')

for i in range(len(coord_selected_)):
    folium.PolyLine(np.array(exp_.coord2gps(coord_selected_[i])).tolist(), color='blue', weight=1.0).add_to(m)



for data1, data2 ,data3, data4 in zip(np.array(est_i10), gps_t, np.array(est_i0), np.array(est_i10_explore)):
    folium.Circle(data2.tolist(), radius=2, color='black', fill=False).add_to(m)
    folium.Circle(data1.tolist(), radius=2, color='orange', fill=False).add_to(m)
    folium.Circle(data3.tolist(), radius=2, color='green', fill=False).add_to(m)
    folium.Circle(data4.tolist(), radius=2, color='magenta', fill=False).add_to(m)
folium.PolyLine(np.array(exp_.coord2gps(coord_selected)).tolist(), color='red', weight=3.0).add_to(m)
m
m.save('output/map_for_paper.html')