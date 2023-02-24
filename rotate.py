import temp
from calc_gps import CalcGPS
from init import Init
import numpy as np
import gps2coord, coord2gps
from calc_traj import CalcTraj
import matplotlib.pyplot as plt
from optimize_traj7 import OptimizeTraj
from extract_roadnet import Extractmap
from temp import ExtractRoads
import folium
import time
import overpy




class rotate():
    def __init__(self):
        self.groundtruth = CalcTraj().calcGroundTruth(Init().N0, Init().M0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, Init().L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, Init().json_file0)
        self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, Init().droid)
        self.optimized = OptimizeTraj().calcOptimizeTraj()
        #self.road = Extractmap(extract_dist=self.EXTRACT_DIST, est=self.optimized).calcDistance()[1]
        self.gps_t = Init().gps_t


    def draw(self):
        pro = np.arange(Init().n_frame)
        print(np.zeros(Init().n_frame))
        pro_ = np.vstack([pro, pro]).T
        droid_ = np.vstack([self.droidslam[0], self.droidslam[1], np.zeros(Init().n_frame)]).T
        pro_ = pro_ - pro_.mean(axis=0)
        #droid_ = droid_ - droid_.mean(axis=0)
        U, S, V = np.linalg.svd(droid_.T @ pro_)
        R_ = np.array([[np.cos(np.pi/4), np.sin(np.pi/4), 0],[-np.sin(np.pi/4), np.cos(np.pi/4), 0], [0, 0, 1]])

        droid_new_ = []
        for i in range(Init().n_frame):
            
            droid_new_.append(np.array((R_ @ droid_[i].T)).T)
        droid_new_ = np.array(droid_new_)
        #R = V.T @ U.T
        #droid_new = (R @ droid_.T).T
        #optimized_rot = self.optimized 
        fig, traj = plt.subplots()
        #traj.plot(groundtruth[1], groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        #traj.plot(pro_[1], pro_[0], color="red", lw=2, label="propo")
        traj.plot(droid_new_.T[1], droid_new_.T[0], color="green", lw=2, label="DROID-SLAM_NEW")
        traj.plot(self.droidslam[1], self.droidslam[0], color="blue", lw=2, label="DROID-SLAM")
        traj.plot(pro_.T[1], pro_.T[0], color="red", lw=2, label="propo")
        #traj.plot(self.optimized[1], self.optimized[0], color="magenta", lw=4, label="Optimized")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Trajectory")
        plt.grid(True)
        plt.savefig('output/opted/trajectory_.png')
        #print(droid_new, "droidnew")
        print(pro_, "pro")
        print(droid_, "droid_old")
        #plt.show()
        
rotate().draw()