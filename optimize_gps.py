#import temp
from calc_gps import CalcGPS
from init import Init
import numpy as np
import gps2coord, coord2gps
from calc_traj import CalcTraj
import matplotlib.pyplot as plt
from optimize_traj7 import OptimizeTraj
#from extract_roadnet import Extractmap
#from temp import ExtractRoads
import folium
import time
import overpy
#from explore_gps import ExploreGPS


class OptimizeGPS():
    def __init__(self, extract_dist):
        self.EXTRACT_DIST = extract_dist
        self.groundtruth = CalcTraj().calcGroundTruth(Init().N0, Init().M0)
        self.L0 = Init().L0
        self.droid = Init().droid
        self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, self.droid)
        if (len(self.L0) == 0):
            self.orbslam = self.droidslam
        else:
            self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, Init().json_file0)
        self.optimized = OptimizeTraj().calcOptimizeTraj()
        #self.road = Extractmap(extract_dist=self.EXTRACT_DIST, est=self.optimized).calcDistance()[1]
        self.gps_t = Init().gps_t
        self.gps_t_ = np.loadtxt('exploreGPS.csv', encoding="shift_jisx0213", delimiter=',')
        #self.gps_t_ = self.gps_t
        

    

    #任意のestの緯度経度変換
    def latlon2coord(self, est):
        coord_t = []
        #print(len(est), "lenest")
        #print(est, "est")
        for i in range(len(np.array(est))):
            coord = gps2coord.calc_xy(np.array(est)[i][0], np.array(est)[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_t.append([coord[0], coord[1]])
        return coord_t

    def coord2gps(self, est):
        gps_est = []
        for i in range(len(np.array(est).T[0])):
            gps = coord2gps.calc_lat_lon(np.array(np.array(est).T[0])[i], np.array(np.array(est).T[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        #print(gps_est)
        return gps_est
    def integratePoints_from_csv(self):
        coord_map = np.loadtxt('extracted_roads.csv', encoding="shift-jis", delimiter=',')
        return coord_map

    #roadの平面直交変換
    def integratePoints(self):
        integrated = []
        coord_map = []
        total = 0
        gps_map = np.array(ExtractRoads(self.EXTRACT_DIST).make_latlon_frommap())
        #m = folium.Map(location=a.gps_t[0], zoom_start=20)
        #folium.PolyLine(gps_map, color='magenta', weight=1).add_to(m)
        #m
        #m.save('output/map_road.html')
        #print(len(gps_map), "lengps_map")
        for i in range(len(gps_map)):
        #    total = total + gps_map[i].size
            for j in gps_map[i]:
                integrated.append(j)
                coord = gps2coord.calc_xy(j[0], j[1], self.gps_t[0][0], self.gps_t[0][1])
                coord_map.append([coord[0], coord[1]])
        return np.array(coord_map)



    def rot2coord(self, est, road):
        f = open('test_list.txt', 'r')
        name_data = f.read().splitlines()
        f.close()
        #print(road, road)
        #est_ = [np.array(est[0]), np.array(est[1])]
        #x_ = np.vstack([np.array(est).T[0], np.array(est).T[1]]).T
        #print(est)
        
        
        #print(x_.shape, "x_")

        y_ = np.vstack([np.array(road).T[0][:], np.array(road).T[1][:]]).T
        y__ = y_.copy()
        y_ = y_ - y_.mean(axis=0)
        #print(y_.shape, "y_")
        try:
            x_ = np.vstack([np.array(est)[1], np.array(est)[0]]).T
            x_ = x_ - x_.mean(axis=0)
#            U1, S, V1 = np.linalg.svd(x_.T @ y_)
            U1, S, V1T = np.linalg.svd(y_.T @ x_)
#            R1 = V1.T @ U1.T
            R1 = U1 @ V1T
            R_det = np.linalg.det(R1)
            sigma = np.array([[1, 0], [0, R_det]])
            R_ = U1 @ sigma @ V1T
            R_det = np.linalg.det(R_)
            
            #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        except ValueError:

            x_ = np.vstack([np.array(est).T[1], np.array(est).T[0]]).T
            x_ = x_ - x_.mean(axis=0)
#            U1, S, V1 = np.linalg.svd(x_.T @ y_)
            U1, S, V1T = np.linalg.svd(y_.T @ x_)
#            R1 = V1.T @ U1.T
            R1 = U1 @ V1T
            R_det = np.linalg.det(R1)
            sigma = np.array([[1, 0], [0, R_det]])
            R_ = U1 @ sigma @ V1T
            R_det = np.linalg.det(R_)
            #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        coord_est = [(R_ @ x_.T)[0] + y__.mean(axis=0)[0], (R_ @ x_.T)[1] + y__.mean(axis=0)[1]]
        return coord_est


    def convert2Coord(self, gps):
        coord_smrc = []
        for i in range(self.n_len):
            coord = gps2coord.calc_xy(gps[i][0], gps[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_smrc.append([coord[0], coord[1]])
        return np.array(coord_smrc)

    def calcDistance(self, coord_smrc):
        coord_map = self.integratePoints_from_csv()
        #print(len(coord_map), "lencoordmap")
        #coord_smrc = np.array(self.rot2coord(est, coord_map))
        
        min = []
        dist_min = []
        #print("coord_smrc",coord_smrc)
        #print("coord_map",coord_map)
        #print(np.array(coord_smrc).T)
        #print(len(coord_smrc.T), "coord_smrc")
        for i in np.array(np.array(coord_smrc)):
            dist = []
            for j in coord_map:
                dist.append(np.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2))
            min.append(coord_map[np.argmin(dist)])
            dist_min.append([np.min(dist), i, coord_map[np.argmin(dist)]])
        dist_min_sorted = np.array(dist_min)[np.argsort(np.array(dist_min).T[0]), :]
        #print(np.array(min), "min")
        return dist_min_sorted, np.array(min).T
    

    '''
    def rotateTraj(self, est):
        
        #if (num > len(self.gps_map)):
        #    coord_map_selected = np.array(self.calcDistance()[0])
        #else:
        #    coord_map_selected = np.array(self.calcDistance()[0])#[:num]
        
        coord_map_selected = np.array(self.calcDistance(est)[1])
        #rotate
        x_ = np.vstack([est[0][0:self.n_len-1], est[1][0:self.n_len-1]]).T
        x_ = x_ - x_.mean(axis=0)
        y_ = np.vstack([coord_map_selected[0][0:self.n_len-1], coord_map_selected[1][0:self.n_len-1]]).T
        y__ = y_ - y_.mean(axis=0)
        #U, S, V = np.linalg.svd(x_.T @ y__)
        U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y__, axis=0))
        R = V.T @ U.T
        coord_est = [(R @ x_.T)[0] + y_.mean(axis=0)[0], (R @ x_.T)[1] + y_.mean(axis=0)[1]]
        

        return coord_est
    '''

    def coord2gps_est(self, est):
        gps_est = []
        coord_est_ = est
        for i in range(len(coord_est_[0])):
            gps = coord2gps.calc_lat_lon(np.array(coord_est_[0])[i], np.array(coord_est_[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        #print(gps_est)
        return gps_est



    def optimizeGPS(self, est, num):
            
        for i in range(num):
            if (i == 0):
                #print(self.gps_t, "gps_t")
                road_coord = self.latlon2coord(self.gps_t_)
                #print(road_coord)
                est_coord_rot = np.array(self.rot2coord(est, road_coord))
                #print(est_coord_rot, "est_coord_rot")
                est_latlon = np.array(self.coord2gps_est(est_coord_rot))
                est_latlon_2 = est_latlon.copy()
                #print(est_latlon_, "est_latlon")
                
                m = folium.Map(location=self.gps_t_[0], zoom_start=20)
                folium.PolyLine(self.gps_t_, color='black', weight=2).add_to(m)
                folium.PolyLine(est_latlon_2, color='magenta', weight=2).add_to(m)
                m
                m.save('output/map_gps.html')
                
                map = folium.Map(location=self.gps_t_[0], zoom_start=20)
                for data1, data2 in zip(np.array(est_latlon_2), self.gps_t_):
                    folium.Circle(data2.tolist(), radius=3, color='black', fill=False).add_to(map)
                    folium.Circle(data1.tolist(), radius=1, color='magenta', fill=False).add_to(map)
                map
                map.save('output/i0.html')
                est_i0 = est_latlon_2.copy()
                np.save('i0_explore_list', est_i0)
                

                print("i=0 completed")
            else:
                #print(est_latlon_2)
                est_coord__ = self.latlon2coord(est_latlon_2)
                #print("est_coord", est_coord__, i)
                #print(self.integratePoints(), "integratepoints")
                #print(np.array(self.calcDistance(est_coord__, est_coord__)[1]), "calcdistance", i)
                #time.sleep(360)
                try:
                    road_coord = np.array(self.calcDistance(est_coord__)[1]).T
                except overpy.exception.OverpassTooManyRequests:
                    break
                except overpy.exception.OverpassGatewayTimeout:
                    break
                #print("road_coord", road_coord, i)
                est_coord_rot_ = np.array(self.rot2coord(est, road_coord))
                
                #print("est_coord_rot", est_coord_rot_)
                road_gps = np.array(self.coord2gps_est(road_coord.T))
                est_latlon_2 = np.array(self.coord2gps_est(est_coord_rot_))
                #est_coord_ = self.latlon2coord(est_latlon_)
                #print(road_gps, "road_gps")
                #m = folium.Map(location=self.gps_t[0], zoom_start=20)
                #for data in road_gps:
                #    folium.Circle(data.tolist(), radius=0.1, color='red', fill=False).add_to(m)
                #m
                #m.save('output/map_road.html')
                #print(est_latlon_2, "est_latlon_",)
                print("i=", i, "completed")
                map = folium.Map(location=self.gps_t_[0], zoom_start=20)
                folium.PolyLine(self.gps_t_, color='black', weight=2).add_to(map)
                folium.PolyLine(est_latlon_2, color='magenta', weight=2).add_to(map)
                map
                map.save('output/map_rotated.html')
                np.save('i10_explore_list', est_latlon_2)
        return est_latlon_2, road_gps, est_i0

'''
a = OptimizeGPS(extract_dist=100)
print(a.gps_t)
output = a.optimizeGPS(a.optimized, 10)
#print(a.optimizeGPS(a.optimized, 1))

m = folium.Map(location=a.gps_t[0], zoom_start=20)
folium.PolyLine(output, color='magenta', weight=1).add_to(m)
m
m.save('output/map_rotated.html')
'''

