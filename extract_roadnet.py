from temp import ExtractRoads
from calc_gps import CalcGPS
from init import Init
import numpy as np
import gps2coord, coord2gps
from calc_traj import CalcTraj
import matplotlib.pyplot as plt

class Extractmap():

    def __init__(self, extract_dist, est):
        self.EXTRACT_DIST = extract_dist
        self.EST = est
        self.gps_t = Init().gps_t
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.L0 = Init().L0
        self.json_file0 = Init().json_file0
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        self.gps_slam = np.array(CalcGPS().coord2gps_est(self.orbslam))
        self.gps_sfm = np.array(CalcGPS().coord2gps_est(self.opensfm))
        self.gps_map = np.array(ExtractRoads(extract_dist=self.EXTRACT_DIST, est=self.EST).make_latlon_frommap())
        self.n_len = Init().n_frame

    def integratePoints(self):
        integrated = []
        coord_map = []
        total = 0
        for i in range(len(self.gps_map)):
            total = total + self.gps_map[i].size
            for j in self.gps_map[i]:
                integrated.append(j)
                coord = gps2coord.calc_xy(j[0], j[1], self.gps_t[0][0], self.gps_t[0][1])
                coord_map.append([coord[0], coord[1]])
        #print(np.array(coord_map).shape)
        return np.array(coord_map)


    def convert2Coord(self, gps):
        coord_smrc = []
        for i in range(self.n_len):
            coord = gps2coord.calc_xy(gps[i][0], gps[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_smrc.append([coord[0], coord[1]])
        return np.array(coord_smrc)

    @staticmethod
    def rot2coord(self, est):
        f = open('test_list.txt', 'r')
        name_data = f.read().splitlines()
        f.close()
        quotient = len(CalcGPS().groundtruth[1]) // len(name_data)
        remainder = len(CalcGPS().groundtruth[1]) % len(name_data)
        est_ = [np.array(est[0]), np.array(est[1])]
        x_ = np.vstack([est_[1], est_[0]]).T
        x_ = x_ - x_.mean(axis=0)
        y_ = np.vstack([np.array(CalcGPS().calcCoord(self)).T[0], np.array(CalcGPS().calcCoord(self)).T[1]]).T
        y__ = y_ - y_.mean(axis=0)
        U, S, V = np.linalg.svd(x_.T @ y__)
        #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        R = V.T @ U.T
        coord_est = [(R @ x_.T)[0] + y_.mean(axis=0)[0], (R @ x_.T)[1] + y_.mean(axis=0)[1]]
        return coord_est

    def calcDistance(self):
        coord_smrc = self.rot2coord(self, self.opensfm)
        coord_map = self.integratePoints()
        min = []
        dist_min = []
        #print(np.array(coord_smrc).T)
        #print(coord_map)
        for i in np.array(coord_smrc).T:
            dist = []
            for j in coord_map:
                dist.append(np.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2))
            min.append(coord_map[np.argmin(dist)])
            dist_min.append([np.min(dist), i, coord_map[np.argmin(dist)]])
        #print("min", np.array(min))
        dist_min_sorted = np.array(dist_min)[np.argsort(np.array(dist_min).T[0]), :]
        
        return dist_min_sorted, np.array(min).T#np.array(dist_min).T[2]
    
    
    
    def show(self):
        coord_sfm = self.rot2coord(self, self.opensfm)
        coord_t = CalcGPS().calcCoord(self)
        coord_orb = self.rot2coord(self, self.orbslam)
        coord_road = self.calcDistance()[1]
        #coord_road = self.integratePoints().T
        fig, traj = plt.subplots()
        #print(coord_t[0])
        traj.plot(np.array(coord_t).T[0], np.array(coord_t).T[1], color="black", lw=0.5, label="estimated")
        traj.plot(coord_sfm[0], coord_sfm[1], color="red", lw=0.5, label="sfm")
        traj.plot(coord_orb[0], coord_orb[1], color="green", lw=0.5, label="orb")
        traj.plot(coord_road[0], coord_road[1], color="magenta", lw=0.5, label="road")
        #traj.plot(coord_map_selected.T[1].T[0], coord_map_selected.T[1].T[1], color="red", lw=0.5, label="pre")
        #traj.plot(coord_map_selected.T[2].T[0], coord_map_selected.T[2].T[1], color="blue", lw=0.5, label="map_extracted")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        plt.grid(True)
        #plt.show()

    
    def rotateTraj(self, est):
        '''
        if (num > len(self.gps_map)):
            coord_map_selected = np.array(self.calcDistance()[0])
        else:
            coord_map_selected = np.array(self.calcDistance()[0])#[:num]
        '''
        coord_map_selected = np.array(self.calcDistance()[1])
        #rotate
        #print(coord_map_selected.T[1])
        x_ = np.vstack([est[0][0:self.n_len-1], est[1][0:self.n_len-1]]).T
        x_ = x_ - x_.mean(axis=0)
        y_ = np.vstack([coord_map_selected[0][0:self.n_len-1], coord_map_selected[1][0:self.n_len-1]]).T
        y__ = y_ - y_.mean(axis=0)
        #U, S, V = np.linalg.svd(x_.T @ y__)
        U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y__, axis=0))
        R = V.T @ U.T
        coord_est = [(R @ x_.T)[0] + y_.mean(axis=0)[0], (R @ x_.T)[1] + y_.mean(axis=0)[1]]
        
        fig, traj = plt.subplots()
        traj.plot(coord_est[0], coord_est[1], color="black", lw=0.5, label="estimated")
        traj.plot(coord_map_selected.T[1].T[0], coord_map_selected.T[1].T[1], color="red", lw=0.5, label="pre")
        traj.plot(coord_map_selected.T[2].T[0], coord_map_selected.T[2].T[1], color="blue", lw=0.5, label="map_extracted")
        traj.set_aspect('equal')
        plt.grid(True)
        plt.savefig('output/opted/coord.png')
        #plt.show()
        return coord_est
    '''
    def rot2coord_map(self, est):
        f = open('test_list.txt', 'r')
        name_data = f.read().splitlines()
        f.close()
        est_ = [np.array(est[0]), np.array(est[1])]
        x_ = np.vstack([est_[1], est_[0]]).T
        x_ = x_ - x_.mean(axis=0)
        y_ = np.vstack([np.array(CalcGPS().calcCoord(self)).T[0], np.array(CalcGPS().calcCoord(self)).T[1]]).T
        R = self.rotateTraj(100)
        coord_est = [(R @ x_.T)[0] + y_.mean(axis=0)[0], (R @ x_.T)[1] + y_.mean(axis=0)[1]]
        return coord_est
    '''

    def coord2gps_est(self, est):
        gps_est = []
        coord_est_ = self.rotateTraj(est)
        for i in range(len(coord_est_[0])):
            gps = coord2gps.calc_lat_lon(np.array(coord_est_[0])[i], np.array(coord_est_[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        #print(gps_est)
        return gps_est

#Extractmap().show()