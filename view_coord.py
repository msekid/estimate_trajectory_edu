import matplotlib.pyplot as plt
import matplotlib
plt.rcParams["font.family"] = "Times New Roman"
import matplotlib
import matplotlib as mpl
import csv
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import numpy as np
from calc_traj import CalcTraj
import glob
from equalize_est import Equalize

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
    
    def showTrajectory(self, groundtruth, opensfm, orbslam, droidslam, optimized, equalizedORB, equalizedDROID):
        fig, traj = plt.subplots(figsize=(32, 32))
        #traj.plot(groundtruth[1], groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        traj.plot(opensfm[11], opensfm[12], color="red", lw=2, label="OpenSfM")
        traj.plot(orbslam[12], orbslam[13], color="green", lw=2, label="ORB-SLAM2")
        traj.plot(droidslam[11], droidslam[12], color="blue", lw=2, label="DROID-SLAM")
        traj.plot(optimized[0], optimized[1], color="magenta", lw=4, label="Optimized")
        traj.plot(equalizedORB[0], equalizedORB[1], color="lightgreen", lw=2, label="equalized ORB-SLAM2")
        traj.plot(equalizedDROID[0], equalizedDROID[1], color="cyan", lw=2, label="equalized DROID-SLAM")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Trajectory")
        plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(True)
        plt.savefig('output/opted/trajectory.png')
        #plt.show()
    
    def showZ(self, groundtruth, opensfm, orbslam, droidslam, optimized):
        fig, traj = plt.subplots()
        time = CalcTraj().Nx
        #traj.plot(groundtruth[1], groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        traj.plot(opensfm[10], opensfm[13][1:], color="red", lw=2, label="OpenSfM")
        traj.plot(orbslam[11], orbslam[14][1:], color="green", lw=2, label="ORB-SLAM2")
        traj.plot(droidslam[10], droidslam[13][1:], color="blue", lw=2, label="DROID-SLAM")
        traj.plot(optimized[6], optimized[2][1:], color="magenta", lw=4, label="Optimized")
        #traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Z")
        plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(True)
        plt.savefig('output/opted/z.png')
        #plt.show()
    
    def showRoll(self, groundtruth, opensfm, orbslam, droidslam, optimized, equalizedORB, equalizedDROID):
        fig, roll = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #roll.plot(CalcTraj().time_groundtruth, groundtruth[2], color="black", lw=0.5, label="Ground Truth")
        #print(opensfm[4][0], "roll_opensfm")
        roll.plot(time, opensfm[2], color="red", lw=2, label="OpenSfM")
        roll.plot(time, droidslam[3], color="blue", lw=2, label="DROID-SLAM")
        roll.plot(time, orbslam[3], color="green", lw=2, label="ORB-SLAM2")
        roll.plot(time[:-1], optimized[3][:-1], color="magenta", lw=4, label="Optimized")
        roll.plot(time, equalizedORB[3], color="lightgreen", lw=2, label="equalized ORB-SLAM2")
        roll.plot(time, equalizedDROID[3], color="cyan", lw=2, label="equalized DROID-SLAM")
        roll.legend(fancybox=False, shadow=False, edgecolor='black')
        roll.set_xlabel("Time [s]")
        roll.set_ylabel("Roll angle [deg]")
        roll.set_title("Roll angle")
        plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(True)
        plt.savefig('output/opted/roll.png')
        #plt.show()
    
    def showPitch(self, groundtruth, opensfm, orbslam, droidslam, optimized, equalizedORB, equalizedDROID):
        fig, pitch = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #pitch.plot(CalcTraj().time_groundtruth, groundtruth[3], color="black", lw=0.5, label="Ground Truth")
        #print(opensfm[3][0], "pitch_opensfm")
        pitch.plot(time, opensfm[3], color="red", lw=2, label="OpenSfM")
        pitch.plot(time, droidslam[4], color="blue", lw=2, label="DROID-SLAM")
        pitch.plot(time, orbslam[4], color="green", lw=2, label="ORB-SLAM2")
        pitch.plot(time[:-1], optimized[4][:-1], color="magenta", lw=4, label="Optimized")
        pitch.plot(time, equalizedORB[4], color="lightgreen", lw=2, label="equalized ORB-SLAM2")
        pitch.plot(time, equalizedDROID[4], color="cyan", lw=2, label="equalized DROID-SLAM")
        pitch.legend(fancybox=False, shadow=False, edgecolor='black')
        pitch.set_xlabel("Time [s]")
        pitch.set_ylabel("Pitch angle [deg]")
        pitch.set_title("Pitch angle")
        plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(True)
        plt.savefig('output/opted/pitch.png')
        #plt.show()

    def showYaw(self, groundtruth, opensfm, orbslam, droidslam, optimized, equalizedORB, equalizedDROID):
        fig,yaw = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        #time2x = N2x
        #yaw.plot(CalcTraj().time_groundtruth, groundtruth[4], color="black", lw=0.5, label="Ground Truth")
        #print(opensfm[2][0], "yaw_opensfm")
        yaw.plot(time, opensfm[4], color="red", lw=2, label="OpenSfM")
        yaw.plot(time, droidslam[5], color="blue", lw=2, label="DROID-SLAM")
        yaw.plot(time, orbslam[5], color="green", lw=2, label="ORB-SLAM2")
        yaw.plot(time[:-1], optimized[5][:-1], color="magenta", lw=4, label="Optimized")
        yaw.plot(time, equalizedORB[5], color="lightgreen", lw=2, label="equalized ORB-SLAM2")
        yaw.plot(time, equalizedDROID[5], color="cyan", lw=2, label="equalized DROID-SLAM")
        yaw.legend(fancybox=False, shadow=False, edgecolor='black')
        yaw.set_xlabel("Time [s]")
        yaw.set_ylabel("Yaw angle [deg]")
        yaw.set_title("Yaw angle")
        plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(True)
        plt.savefig('output/opted/yaw.png')
        #plt.show()