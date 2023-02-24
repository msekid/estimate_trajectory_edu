from calc_traj import CalcTraj
import gps2coord, coord2gps
import numpy as np
import glob
from scipy import integrate, signal, interpolate
from init import Init
from calc_traj import CalcTraj
import matplotlib.pyplot as plt

class CalcGPS():
    def __init__(self):
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.gps_t = Init().gps_t
        self.L0 = Init().L0
        self.json_file0 = Init().json_file0
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)

    @staticmethod
    def calcCoord(self):
        coord_t = []
        for i in range(len(self.gps_t)):
            coord = gps2coord.calc_xy(self.gps_t[i][0], self.gps_t[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_t.append([coord[0], coord[1]])
        return coord_t
    
    @staticmethod
    def rot2coord(self, est):
        f = open('test_list.txt', 'r')
        name_data = f.read().splitlines()
        f.close()
        quotient = len(CalcGPS().groundtruth[1]) // len(name_data)
        remainder = len(CalcGPS().groundtruth[1]) % len(name_data)
        est_ = [np.array(est[0]), np.array(est[1])]
        x_ = np.vstack([est_[1][0:len(name_data)-1], est_[0][0:len(name_data)-1]]).T
        x_ = x_ - x_.mean(axis=0)
        y_ = np.vstack([np.array(CalcGPS().calcCoord(self)).T[0][0:len(name_data)-1], np.array(CalcGPS().calcCoord(self)).T[1][0:len(name_data)-1]]).T
        y__ = y_ - y_.mean(axis=0)
        U, S, V = np.linalg.svd(x_.T @ y__)
        #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        R = V.T @ U.T
        coord_est = [(R @ x_.T)[0] + y_.mean(axis=0)[0], (R @ x_.T)[1] + y_.mean(axis=0)[1]]
        return coord_est
    
    def coord2gps_est(self, est):
        gps_est = []
        coord_est_ = CalcGPS().rot2coord(self, est)
        for i in range(len(coord_est_[0])):
            gps = coord2gps.calc_lat_lon(np.array(coord_est_[0])[i], np.array(coord_est_[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        #print(gps_est)
        return gps_est
    
    def show(self):
        coord_sfm = self.rot2coord(self, self.opensfm)
        coord_t = self.calcCoord(self)
        coord_orb = self.rot2coord(self, self.orbslam)
        fig, traj = plt.subplots()
        print(coord_t[0])
        traj.plot(np.array(coord_t).T[0], np.array(coord_t).T[1], color="black", lw=0.5, label="estimated")
        traj.plot(coord_sfm[0], coord_sfm[1], color="red", lw=0.5, label="sfm")
        traj.plot(coord_orb[0], coord_orb[1], color="green", lw=0.5, label="orb")
        #traj.plot(coord_map_selected.T[1].T[0], coord_map_selected.T[1].T[1], color="red", lw=0.5, label="pre")
        #traj.plot(coord_map_selected.T[2].T[0], coord_map_selected.T[2].T[1], color="blue", lw=0.5, label="map_extracted")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        plt.grid(True)
        plt.show()

#CalcGPS().coord2gps_est(CalcGPS().opensfm)
#CalcGPS().show()