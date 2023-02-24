from lib2to3.pgen2.token import OP
from pickletools import optimize
from urllib.request import CacheFTPHandler
import numpy as np
from scipy import integrate, signal, interpolate
from calc_traj import CalcTraj
from calc_RT__ import CalcRT
from init import Init
import math
import matplotlib.pyplot as plt
import time
class OptimizeTraj():
    def __init__(self):
        self.L0 = Init().L0
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.time = Init().Nx
        self.t_orb = Init().L0[:, 0]
        self.json_file0 = open('reconstruction.json', 'r')
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        #print(time.time()-start)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        #print(time.time()-start)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        #print(time.time()-start)
        self.orbRT = CalcRT().calcRT_orbslam_interpolated()
        #print(time.time()-start)
        self.RTonly = CalcRT().calc_RT_only()
        #print(time.time()-start)
        self.sfmRT = CalcRT().calcRT_sfm()
        #print(time.time()-start)

    def rotateSfM(self):
        t_sfm = np.array(self.opensfm[7])
        R_sfm = np.array(self.opensfm[6])
        R_sfm__ = R_sfm.copy()
        R_sfm_ = []
        for i in range(len(R_sfm)):
            R_sfm_.append(R_sfm[i].T)
        
        t_slam = np.array(self.orbRT[2])


        t_sfm = t_sfm - t_sfm.mean(axis=0)
        t_slam = t_slam - t_slam.mean(axis=0)
        #U, S, V = np.linalg.svd(t_sfm.T @ t_slam)
        U, S, V = np.linalg.svd(np.diff(t_sfm[:-1], axis=0).T @ np.diff(t_slam[:-1], axis=0))
        R_ = V.T @ U.T
        t_sfm_new = (R_ @ t_sfm.T).T
        R_sfm_new = R_sfm
        for i in range(len(R_sfm)):
            R_sfm_new[i] = R_sfm_[i]
            #R_sfm_new[i] = R_sfm[i]
        
        v_sfm = []
        for i in range(len(t_sfm)-1):
            #v = t_sfm_new[i+1] - np.dot(R_sfm_new[i+1], np.dot(R_sfm_new[i].T, t_sfm_new[i]))
            v = R_sfm_new[i+1] @ (t_sfm_new[i+1]-t_sfm_new[i])
            v_norm = v/np.linalg.norm(v)
            #print("v_sfm", i , np.linalg.norm(v))
            v_sfm.append(v)
        '''
        fig, traj = plt.subplots()
        traj.plot(self.groundtruth[1], self.groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        traj.plot(t_sfm[:-1, 1], t_sfm[:-1, 0], color="blue", lw=0.5, label="OpenSfM")
        traj.plot(t_sfm_new[:-1, 1], t_sfm_new[:-1, 0], color="red", lw=0.5, label="OpenSfM")
        traj.plot(t_slam[:-1, 1], t_slam[:-1, 0], color="green", lw=0.5, label="ORB-SLAM2")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Trajectory")
        plt.grid(True)
        plt.savefig('output/opted/trajectory__.png')
        '''

        return t_sfm_new, R_sfm__, v_sfm, R_sfm_new, R_



    def calcQ(self):
        rotateSfM = self.rotateSfM()
        Q_sfm = []
        Q_sfm_r = []
        Q_slam = []
        Q_slam_r = []
        #R_sfm = np.array(CalcRT().calcRT_sfm()[2])
        #R_sfm = np.array(rotateSfM[3])
        R_sfm_r = np.array(rotateSfM[1])
        #R_slam = np.array(CalcRT().calcRT_orbslam_interpolated()[1])
        #R_slam = np.array(self.RTonly[2])
        R_slam_r = np.array(self.orbRT[1])
        #print(R_slam)
        for i in range(len(self.time)):
            #Q_sfm.append(R_sfm[i+1] @ np.linalg.inv(R_sfm[i]))
            if (i == 0):
                #Q_sfm.append(np.array(R_sfm[i]))
                Q_sfm_r.append(np.array(R_sfm_r[i]))
                #Q_slam.append(np.array(R_slam[i]))
                Q_slam_r.append(np.array(R_slam_r[i]))
            else:
                #Q_sfm.append(np.array(R_sfm[i]) @ R_sfm[i-1].T)
                Q_sfm_r.append(np.array(R_sfm_r[i]) @ R_sfm_r[i-1].T)
                #Q_slam.append(R_slam[i] @ np.linalg.inv(R_slam[i-1]))
                Q_slam_r.append(R_slam_r[i] @ np.linalg.inv(R_slam_r[i-1]))
                #Q_sfm.append(np.array(R_sfm[i-1]).T @ R_sfm[i])
                #Q_slam.append(np.linalg.inv(R_slam[i-1]) @ R_slam[i])
        #print("Q[0]=", Q_sfm[0])
        return Q_sfm, Q_slam, Q_sfm_r, Q_slam_r


    def calcWeight(self):
        rotateSfM = self.rotateSfM()
        Q = self.calcQ()
        RT_sfm = np.array(self.sfmRT[0])
        RT_slam = np.array(self.RTonly[0])
        list = np.array(range(len(self.time)))
        R_slam = np.zeros((len(self.time), 3, 3))
        R_slam_r = np.zeros((len(self.time), 3, 3))
        R_sfm = np.zeros((len(self.time), 3, 3))
        R_sfm_r = np.zeros((len(self.time), 3, 3))
        t_slam = np.zeros((len(self.time), 3))
        t_sfm = np.zeros((len(self.time), 3))
        R = np.zeros((len(self.time), 3, 3))
        R_r = np.zeros((len(self.time), 3, 3))
        t = np.zeros((len(self.time), 3))
        #print(R.shape, t.shape)
        #v_sfm_r = np.array(self.sfmRT[1])
        
        v_sfm_r = np.array(rotateSfM[2])
        v_slam = np.array(self.orbRT[0])
        #v_slam = np.array(self.RTonly[1])
        Q_sfm = Q[0]
        Q_sfm_r = Q[2]
        Q_slam = Q[1]
        Q_slam_r = Q[3]
        R_ = rotateSfM[4]
        for i,j in zip(self.time, list[1:]):
            if (j == 1):
                #R_slam[j-1] = np.array(Q_slam[j-1])
                R_slam_r[j-1] = np.array(Q_slam_r[j-1])
                #R_sfm[j-1] = np.array(Q_sfm[j-1])
                R_sfm_r[j-1] = np.array(Q_sfm_r[j-1])
            else:
                #R_slam[j-1] = np.array(Q_slam[j-1]) @ np.array(R_slam[j-2])
                R_slam_r[j-1] = np.array(Q_slam_r[j-1]) @ np.array(R_slam_r[j-2])
                t_slam[j-1] = np.array(t_slam[j-2]) + np.array(R_slam_r[j-1]) @ np.array(v_slam[j-1])
                #R_sfm[j-1] = np.array(Q_sfm[j-1]) @ np.array(R_sfm[j-2])
                R_sfm_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_sfm_r[j-2])
                t_sfm[j-1] = np.array(t_sfm[j-2]) + np.array(R_sfm_r[j-1]) @ np.array(v_sfm_r[j-1])

        #print("R[0]=", R_sfm[0])
        for i,j in zip(self.time, list[1:]):
            if (j == 1):
                    #R[j-1] = np.array(Q_sfm[j-1])
                    R_r[j-1] = np.array(Q_sfm_r[j-1])
                    #print(R, "j==1")
            else:
                if (self.t_orb[0] < i or self.t_orb[len(self.t_orb)-1] > i):
                    if (RT_sfm[j-1] < RT_slam[j-1]):
                        #R[j-1] = np.array(Q_sfm[j-1]) @ np.array(R[j-2])
                        R_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_sfm_r[j-1]) @ np.array(v_sfm_r[j-1])
                    else:
                        #R[j-1] = np.array(Q_slam[j-1]) @ np.array(R[j-2])
                        R_r[j-1] = np.array(Q_slam_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_slam_r[j-1]) @ np.array(v_slam[j-1])
                else:
                    #R[j-1] = np.array(Q_sfm[j-1]) @ np.array(R[j-2])
                    R_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_r[j-2])
                    t[j-1] = np.array(t[j-2]) + np.array(R_sfm_r[j-1]) @ np.array(v_sfm_r[j-1])
        
        return R_r, t, t_sfm, t_slam#R_r, t, t_sfm, t_slam

    def calcOptimizeTraj(self):
        xyz_opt = []
        opt = self.calcWeight()
        R_opt = opt[0]
        t_opt = opt[1]
        for i in range(len(self.time)-1):
            #xyz_opt.append(-np.dot(np.array(R_opt[i]).T, np.array(t_opt[i])))
            xyz_opt.append(np.array(t_opt[i]))
        #print(np.array(xyz_opt).T[0])
        #l_orb = 0
        #for n in range(len(self.time) - 1):
        #    l_orb[n] = np.sqrt((xyz_opt_[:, 0][n + 1] - xyz_opt_[:, 0][n]) ** 2 + (xyz_opt_[:, 1][n + 1] - xyz_opt_[:, 1][n]) ** 2)
        #    L_orb = L_orb + l_orb[n]

        #k_v=self.groundtruth[6]/L_orb
        #xyz_opt = k_v*xyz_opt_

        #print(R_opt)
        R4 = np.zeros((len(self.time)-1, 3, 3))
        r1 = np.zeros(len(self.time))
        p1 = np.zeros(len(self.time))
        ya1 = np.zeros(len(self.time))
        for i in range(len(self.time)-1):
            """
            R4[i+1] = R_opt[i] @ np.linalg.inv(R_opt[i+1])
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R4[i+1]))
            r1[i+1] = eul[0] + r1[i]
            p1[i+1] = eul[1] + p1[i]
            ya1[i+1] = eul[2] + ya1[i]
            """
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R_opt[i]))
            r1[i] = -eul[0]
            p1[i] = eul[1] % 360 - 180
            ya1[i] = -eul[2]
            

        return np.array(xyz_opt)[:, 0], np.array(xyz_opt)[:, 1], np.array(xyz_opt)[:, 2],r1-r1[0], p1-p1[0],ya1-ya1[0]
    
    
    
    def showTrajectory(self, opt):
        opt_x = opt[0]
        opt_y = opt[1]
        opted = self.calcWeight()
        opt_sfm = opted[2]
        opt_slam = opted[3]
        #print(opt_x,opt_y)
        fig, traj = plt.subplots()
        traj.plot(self.groundtruth[1], self.groundtruth[0], color="black", lw=0.5, label="Ground Truth")
        traj.plot(opt_y, opt_x, color="red", lw=0.5, label="Optimized")
        traj.plot(opt_sfm[:-1, 1], opt_sfm[:-1, 0], color="blue", lw=0.5, label="OpenSfM")
        traj.plot(opt_slam[:-1, 1], opt_slam[:-1, 0], color="green", lw=0.5, label="ORB-SLAM2")
        traj.set_aspect('equal')
        traj.legend(fancybox=False, shadow=False, edgecolor='black')
        traj.set_ylabel("Depth direction [m]")
        traj.set_xlabel("Lateral direction [m]")
        traj.set_title("Trajectory")
        plt.grid(True)
        plt.savefig('output/opted/trajectory_.png')
        #plt.show()

    def showRoll(self, opt):
        fig, roll = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        opt_roll = opt[5]
        #time2x = N2x
        roll.plot(CalcTraj().time_groundtruth, self.groundtruth[2], color="black", lw=0.5, label="Ground Truth")
        roll.plot(time[:-1], opt_roll[:-1], color="red", lw=0.5, label="Optimized")
        #pitch.plot(time, colmap[3], color="blue", lw=0.5, label="Colmap")
        roll.legend(fancybox=False, shadow=False, edgecolor='black')
        roll.set_xlabel("Time [s]")
        roll.set_ylabel("Roll angle [deg]")
        roll.set_title("Roll angle")
        plt.grid(True)
        plt.savefig('output/opted/roll.png')
        #plt.show()


    def showPitch(self, opt):
        fig, pitch = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        opt_pitch = opt[4]
        #time2x = N2x
        pitch.plot(CalcTraj().time_groundtruth, self.groundtruth[3], color="black", lw=0.5, label="Ground Truth")
        pitch.plot(time[:-1], opt_pitch[:-1], color="red", lw=0.5, label="Optimized")
        #pitch.plot(time, colmap[3], color="blue", lw=0.5, label="Colmap")
        pitch.legend(fancybox=False, shadow=False, edgecolor='black')
        pitch.set_xlabel("Time [s]")
        pitch.set_ylabel("Pitch angle [deg]")
        pitch.set_title("Pitch angle")
        plt.grid(True)
        plt.savefig('output/opted/pitch.png')
        #plt.show()
    
    def showYaw(self, opt):
        fig, Yaw = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        opt_Yaw = opt[3]
        #time2x = N2x
        Yaw.plot(CalcTraj().time_groundtruth, self.groundtruth[4], color="black", lw=0.5, label="Ground Truth")
        Yaw.plot(time[:-1], opt_Yaw[:-1], color="red", lw=0.5, label="Optimized")
        #pitch.plot(time, colmap[3], color="blue", lw=0.5, label="Colmap")
        Yaw.legend(fancybox=False, shadow=False, edgecolor='black')
        Yaw.set_xlabel("Time [s]")
        Yaw.set_ylabel("Yaw angle [deg]")
        Yaw.set_title("Yaw angle")
        plt.grid(True)
        plt.savefig('output/opted/yaw.png')
        #plt.show()

'''
a = OptimizeTraj()
b = a.calcOptimizeTraj()
a.showTrajectory(b)
a.showRoll(b)
a.showPitch(b)
a.showYaw(b)
'''

