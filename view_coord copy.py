import matplotlib.pyplot as plt
import matplotlib
import matplotlib as mpl
import csv
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import numpy as np
from calc_traj import CalcTraj
import glob

class ViewCoord():
    def __init__(self):
        pass
    #    self.N0 = np.loadtxt(glob.glob('ID*')[0], encoding="shift-jis", delimiter=',', skiprows=2, usecols=[2, 39, 40, 41])
    #    self.M0 = np.loadtxt(glob.glob('ID*')[0], encoding="shift-jis", delimiter=',', skiprows=2, usecols=[8, 9, 10, 13])
    #    self.gps_t = np.loadtxt(glob.glob('ID*')[0], encoding="shift-jis", delimiter=',', skiprows=2, usecols=[30, 31])
    #    self.L0 = np.loadtxt('KeyFrameTrajectory.txt', delimiter=' ')
    #    self.json_file0 = open('reconstruction.json', 'r')
    #    self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
    #    self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
    #    self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
    
    def showTrajectory(self, groundtruth, opensfm, orbslam):
        fig, traj = plt.subplots()
        #traj.plot(groundtruth[1], groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        traj.plot(opensfm[1], opensfm[0], color="red", lw=0.5, label="OpenSfM")
        traj.plot(orbslam[1], orbslam[0], color="green", lw=0.5, label="ORB-SLAM2")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Trajectory")
        plt.grid(True)
        plt.savefig('output/plotimage/trajectory.png')
        #plt.show()
    
    def showRoll(self, groundtruth, opensfm, orbslam):
        fig, roll = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #roll.plot(CalcTraj().time_groundtruth, groundtruth[2], color="black", lw=0.5, label="Ground Truth")
        roll.plot(time, opensfm[4], color="red", lw=0.5, label="OpenSfM")
        #roll.plot(time, colmap[2], color="blue", lw=0.5, label="Colmap")
        roll.plot(time, orbslam[3], color="green", lw=0.5, label="ORB-SLAM2")
        roll.legend(fancybox=False, shadow=False, edgecolor='black')
        roll.set_xlabel("Time [s]")
        roll.set_ylabel("Roll angle [deg]")
        roll.set_title("Roll angle")
        plt.grid(True)
        plt.savefig('output/plotimage/roll.png')
        #plt.show()
    
    def showPitch(self, groundtruth, opensfm, orbslam):
        fig, pitch = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #pitch.plot(CalcTraj().time_groundtruth, groundtruth[3], color="black", lw=0.5, label="Ground Truth")
        pitch.plot(time, opensfm[3], color="red", lw=0.5, label="OpenSfM")
        #pitch.plot(time, colmap[3], color="blue", lw=0.5, label="Colmap")
        pitch.plot(time, orbslam[4], color="green", lw=0.5, label="ORB-SLAM2")
        pitch.legend(fancybox=False, shadow=False, edgecolor='black')
        pitch.set_xlabel("Time [s]")
        pitch.set_ylabel("Pitch angle [deg]")
        pitch.set_title("Pitch angle")
        plt.grid(True)
        plt.savefig('output/plotimage/pitch.png')
        #plt.show()

    def showYaw(self, groundtruth, opensfm, orbslam):
        fig,yaw = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #yaw.plot(CalcTraj().time_groundtruth, groundtruth[4], color="black", lw=0.5, label="Ground Truth")
        yaw.plot(time, opensfm[2], color="red", lw=0.5, label="OpenSfM")
        #yaw.plot(time, colmap[4], color="blue", lw=0.5, label="Colmap")
        yaw.plot(time, orbslam[5], color="green", lw=0.5, label="ORB-SLAM2")
        yaw.legend(fancybox=False, shadow=False, edgecolor='black')
        yaw.set_xlabel("Time [s]")
        yaw.set_ylabel("Yaw angle [deg]")
        yaw.set_title("Yaw angle")
        plt.grid(True)
        plt.savefig('output/plotimage/yaw.png')
        #plt.show()