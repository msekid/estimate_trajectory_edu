from lib2to3.pgen2.token import OP
from pickletools import optimize
from urllib.request import CacheFTPHandler
import numpy as np
np.set_printoptions(threshold=np.inf)
from scipy import integrate, signal, interpolate
from calc_traj import CalcTraj
from calc_RT__ import CalcRT
from init import Init
import math
import matplotlib.pyplot as plt
import time
from equalize_est import Equalize

class OptimizeTraj():
    def __init__(self):
        a = Init()
        self.L0 = a.L0
        self.N0 = a.N0
        self.M0 = a.M0
        self.time = a.Nx
        if (len(self.L0) == 0):
            self.t_orb = a.Nx
        else:
            self.t_orb = a.L0[:, 0]
        self.json_file0 = open('reconstruction.json', 'r')
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        #print(time.time()-start)
        self.droid = a.droid
        
        self.droidslam = Equalize().averagedSLAM(a.droid0, a.droid1, a.droid2, a.droid3, a.droid4, a.droid5, a.droid6, a.droid7, a.droid8, a.droid9, 'DROIDSLAM')
        
        if (len(a.L) == 0):
            self.orbslam = self.droidslam
        else:
            #self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
            self.orbslam = Equalize().averagedSLAM(a.L0, a.L1, a.L2, a.L3, a.L4, a.L5, a.L6, a.L7, a.L8, a.L9, 'ORBSLAM')
        #print(time.time()-start)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        #print(time.time()-start)
        #self.droidRT = CalcRT().calcRT_droidslam_interpolated()
        #if (len(self.L0) == 0):
        #    self.orbRT = self.droidRT
        #else:
        #    self.orbRT = CalcRT().calcRT_orbslam_interpolated()
        #print(time.time()-start)
        #print(time.time()-start)
        #self.sfmRT = CalcRT().calcRT_sfm()
        #print(time.time()-start)
        self.output = CalcRT().calcRT(self.orbslam[0], self.orbslam[1], self.orbslam[2], self.orbslam[3], self.orbslam[4], self.orbslam[5])
        
        


    def rotateSfM(self):
        output = CalcRT().calcRT(self.opensfm[0], self.opensfm[1], self.opensfm[9], self.opensfm[2], self.opensfm[3], self.opensfm[4])
        t_sfm = np.array(output[0])
        #t_sfm_ = np.array(self.opensfm[9])
        #R_sfm = np.array(output[2])
        R_sfm2 = np.array(output[2])
        '''
        R_sfm__ = R_sfm.copy()
        R_sfm_ = []
        for i in range(len(R_sfm)):
            R_sfm_.append(R_sfm[i].T)
        R_sfm_new = R_sfm
        for i in range(len(R_sfm)):
            R_sfm_new[i] = R_sfm_[i]

        '''
        t_sfm_new = np.array(output[0])

        '''
        x_ = t_sfm_new - t_sfm_new.mean(axis=0) 
        y_ = np.vstack([self.orbslam[0], self.orbslam[1], self.orbslam[2]]).T
        y_ = y_ - y_.mean(axis=0)
        U, S, V = np.linalg.svd(x_.T @ y_)
        R = V.T @ U.T
        t_sfm_new = (R @ x_.T).T
        '''

        v_sfm = np.array(output[1])
        '''
        v_sfm = []
        for i in range(len(t_sfm)):
            #v = t_sfm_new[i+1] - np.dot(R_sfm_new[i+1], np.dot(R_sfm_new[i].T, t_sfm_new[i]))
            if (i == 0):
                v_sfm.append([0, 0, 0])
            else:
                v = R_sfm2[i] @ (t_sfm_new[i]-t_sfm_new[i-1])
                v_norm = v/np.linalg.norm(v)
                #print("v_sfm", i , np.linalg.norm(v))
                v_sfm.append(v)
        '''
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

        return t_sfm_new, R_sfm2, v_sfm, output[3]#R_sfm_new


    def rotateDroid(self):
        output = CalcRT().calcRT(self.droidslam[0], self.droidslam[1], self.droidslam[2], self.droidslam[3], self.droidslam[4], self.droidslam[5])
        t_sfm_new = output[0]
        R_sfm__ = output[2]
        v_sfm = output[1]
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

        return t_sfm_new, R_sfm__, v_sfm, output[3]


    def calcQ(self):
        rotateSfM = self.rotateSfM()
        rotateDroid = self.rotateDroid()
        Q_sfm = []
        Q_sfm_r = []
        Q_slam = []
        Q_slam_r = []
        Q_droid_r = []
        #R_sfm = np.array(CalcRT().calcRT_sfm()[2])
        #R_sfm = np.array(rotateSfM[3])
        R_sfm_r = np.array(rotateSfM[1])
        #R_slam = np.array(CalcRT().calcRT_orbslam_interpolated()[1])
        #R_slam = np.array(self.RTonly[2])
        
        R_slam_r = np.array(self.output[2])
        R_droid_r = np.array(rotateDroid[1])
        #print(R_slam)
        for i in range(len(self.time)):
            #Q_sfm.append(R_sfm[i+1] @ np.linalg.inv(R_sfm[i]))
            if (i == 0):
                #Q_sfm.append(np.array(R_sfm_r[i]))
                Q_sfm_r.append(np.array(CalcRT().euler2RotationMatrix(np.deg2rad(np.zeros(Init().n_frame)), np.deg2rad(np.zeros(Init().n_frame)), np.deg2rad(np.zeros(Init().n_frame))))[i])
                #print(Q_sfm_r, "i=0")
                #Q_slam.append(np.array(R_slam[i]))
                Q_slam_r.append(np.array(R_slam_r[i]))
                Q_droid_r.append(np.array(R_droid_r[i]))
            else:
                #Q_sfm.append(np.array(R_sfm[i]) @ R_sfm[i-1].T)
                Q_sfm_r.append(np.array(R_sfm_r[i]) @ R_sfm_r[i-1].T)
                #Q_slam.append(R_slam[i] @ np.linalg.inv(R_slam[i-1]))
                Q_slam_r.append(R_slam_r[i] @ np.linalg.inv(R_slam_r[i-1]))
                Q_droid_r.append(R_droid_r[i] @ np.linalg.inv(R_droid_r[i-1]))
                #Q_sfm.append(np.array(R_sfm[i-1]).T @ R_sfm[i])
                #Q_slam.append(np.linalg.inv(R_slam[i-1]) @ R_slam[i])
        #print("Q[0]=", Q_sfm[0])
        return Q_sfm, Q_slam, Q_sfm_r, Q_slam_r, Q_droid_r

    def showRT(self):
        rotateSfM = self.rotateSfM()
        rotateDroid = self.rotateDroid()
        RT_sfm = np.array(rotateSfM[3])
        RT_slam_ = np.array(self.output[3])
        RT_droid = np.array(rotateDroid[3])

        time = Init().Nx
        fig, rt = plt.subplots(figsize=(32, 8))

        rt.plot(time[:], np.array(RT_sfm).T, color="red", lw=2, label="Normalized vehicle attitude from OpenSfM")
        rt.plot(time[:], np.array(RT_slam_).T, color="green", lw=2, label="Normalized vehicle attitude from ORB-SLAM2")
        rt.plot(time[:], np.array(RT_droid).T, color="blue", lw=2, label="Normalized vehicle attitude from DROID-SLAM")
        rt.legend(fancybox=False, shadow=False, edgecolor='black')
        rt.set_xlabel("Time [s]")
        rt.set_ylabel("Normalized vehicle attitude")
        plt.grid(True)
        plt.savefig('output/opted/RT.png')
        
        
    def calcWeight(self):
        rotateSfM = self.rotateSfM()
        rotateDroid = self.rotateDroid()
        Q = self.calcQ()
        RT_sfm = np.array(rotateSfM[3])
        #RT_slam = np.array(self.RTonly[0])
        RT_slam_ = np.array(self.output[3])
        RT_slam = []
        #print(RT_slam_, "before")
        for i in RT_slam_:
            if (np.isnan(i)):
                RT_slam.append(10)
            else:
                RT_slam.append(i)
        #print(RT_slam, "after")
        #RT_droid = np.array(self.RTonly_doid[0])
        RT_droid = np.array(rotateDroid[3])
        list = np.array(range(len(self.time)+1))
        R_slam = np.zeros((len(self.time), 3, 3))
        R_slam_r = np.zeros((len(self.time), 3, 3))
        R_sfm = np.zeros((len(self.time), 3, 3))
        R_sfm_r = np.zeros((len(self.time), 3, 3))
        R_droid_r = np.zeros((len(self.time), 3, 3))
        t_slam = np.zeros((len(self.time), 3))
        t_sfm = np.zeros((len(self.time), 3))
        t_droid = np.zeros((len(self.time), 3))
        R = np.zeros((len(self.time), 3, 3))
        R_r = np.zeros((len(self.time), 3, 3))
        t = np.zeros((len(self.time), 3))
        #print(R.shape, t.shape)
        #v_sfm_r = np.array(self.sfmRT[1])
        
        v_sfm_r = np.array(rotateSfM[2])
        v_slam = np.array(self.output[1])
        #v_slam = np.array(Equalize().equalizeORBSLAM()[1])
        #print(v_sfm_r/np.linalg.norm(v_sfm_r, axis=1, keepdims=True))
        #print(np.hstack([v_sfm_r/np.linalg.norm(v_sfm_r, axis=1, keepdims=True), v_slam/np.linalg.norm(v_slam, axis=1, keepdims=True)]))
        #np.savetxt('hh.csv', np.hstack([v_sfm_r/np.linalg.norm(v_sfm_r, axis=1, keepdims=True), v_slam/np.linalg.norm(v_slam, axis=1, keepdims=True)]), delimiter=",")
        #print(v_slam/np.linalg.norm(v_slam, axis=1, keepdims=True), "v_slam")
        v_droid = np.array(rotateDroid[2])
        Q_sfm_r = Q[2]
        Q_slam_r = Q[3]
        #Q_slam_r = np.array(Equalize().equalizeORBSLAM()[0])
        Q_droid_r = Q[4]
        #print(np.array(R_droid_r).shape)
        for i,j in zip(self.time, list[1:]):
            if (j == 1):
                #R_slam[j-1] = np.array(Q_slam[j-1])
                R_slam_r[j-1] = np.array(Q_slam_r[j-1])
                #R_sfm[j-1] = np.array(Q_sfm[j-1])
                R_sfm_r[j-1] = np.array(Q_sfm_r[j-1])
                R_droid_r[j-1] = np.array(Q_droid_r[j-1])
            else:
                #R_slam[j-1] = np.array(Q_slam[j-1]) @ np.array(R_slam[j-2])
                R_slam_r[j-1] = np.array(Q_slam_r[j-1]) @ np.array(R_slam_r[j-2])
                t_slam[j-1] = np.array(t_slam[j-2]) + np.array(R_slam_r[j-1]) @ np.array(v_slam[j-1])
                
                #R_sfm[j-1] = np.array(Q_sfm[j-1]) @ np.array(R_sfm[j-2])
                R_sfm_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_sfm_r[j-2])
                t_sfm[j-1] = np.array(t_sfm[j-2]) + np.array(R_sfm_r[j-1]) @ np.array(v_sfm_r[j-1])

                R_droid_r[j-1] = np.array(Q_droid_r[j-1]) @ np.array(R_droid_r[j-2])
                t_droid[j-1] = np.array(t_droid[j-2]) + np.array(R_droid_r[j-1]) @ np.array(v_droid[j-1])
                

        #print("R[0]=", R_sfm[0])
        for i,j in zip(self.time, list[1:]):
            if (j == 1):
                    #R_r[j-1] = np.array(Q_sfm_r[j-1])
                    #t[j-1] = np.array(v_slam[j-1])
                    R_r[j-1] = np.array(np.eye(3))
                    t[j-1] = np.array(np.zeros(3))
            else:
                '''
                if (self.t_orb[0] >= i ): #self.t_orb[len(self.t_orb)-1] > i
                    if (RT_sfm[j-1] <= RT_droid[j-1]):
                    #[j-1] = np.array(Q_sfm[j-1]) @ np.array(R[j-2])
                        R_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_sfm_r[j-1])
                    else:
                        R_r[j-1] = np.array(Q_droid_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_droid[j-1])
                elif(self.t_orb[len(self.t_orb)-1] <= i):
                    #print(len(RT_sfm), len(RT_droid),"len")
                    if (RT_sfm[j-1] <= RT_droid[j-1]):
                    #R[j-1] = np.array(Q_sfm[j-1]) @ np.array(R[j-2])
                        R_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_sfm_r[j-1])
                    else:
                        R_r[j-1] = np.array(Q_droid_r[j-1]) @ np.array(R_r[j-2])
                        t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_droid[j-1])
                
                else:
                '''
                if (RT_slam[j-1] < RT_sfm[j-1] and RT_slam[j-1] < RT_droid[j-1]):
                    R_r[j-1] = np.array(Q_slam_r[j-1]) @ np.array(R_r[j-2])
                    t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_slam[j-1])
                elif (RT_droid[j-1] < RT_slam[j-1] and RT_droid[j-1] < RT_sfm[j-1]):
                    R_r[j-1] = np.array(Q_droid_r[j-1]) @ np.array(R_r[j-2])
                    t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_droid[j-1])
                else:
                    R_r[j-1] = np.array(Q_sfm_r[j-1]) @ np.array(R_r[j-2])
                    t[j-1] = np.array(t[j-2]) + np.array(R_r[j-1]) @ np.array(v_sfm_r[j-1])
                    
        #print(R_r[0])
        #print(v_sfm_r, v_slam)
        return R_r, t , t_sfm, t_slam#R_r, t, t_sfm, t_slam

    def calcOptimizeTraj(self):
        xyz_opt = []
        opt = self.calcWeight()
        R_opt = opt[0]
        t_opt = np.array(opt[1])
        for i in range(len(self.time)):
            #xyz_opt.append(-np.dot(np.array(R_opt[i]).T, np.array(t_opt[i])))
            xyz_opt.append(np.array(t_opt[i]))
        #print(np.array(xyz_opt).T[0])
        #l_orb = 0
        #for n in range(len(self.time) - 1):
        #    l_orb[n] = np.sqrt((xyz_opt_[:, 0][n + 1] - xyz_opt_[:, 0][n]) ** 2 + (xyz_opt_[:, 1][n + 1] - xyz_opt_[:, 1][n]) ** 2)
        #    L_orb = L_orb + l_orb[n]

        #k_v=self.groundtruth[6]/L_orb
        #xyz_opt = k_v*xyz_opt_
        np.save('np_save', R_opt)
        #print(R_opt)
        R4 = np.zeros((len(self.time)-1, 3, 3))
        r1 = np.zeros(len(self.time))
        p1 = np.zeros(len(self.time))
        ya1 = np.zeros(len(self.time))
        for i in range(len(self.time)):
            """
            R4[i+1] = R_opt[i] @ np.linalg.inv(R_opt[i+1])
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R4[i+1]))
            r1[i+1] = eul[0] + r1[i]
            p1[i+1] = eul[1] + p1[i]
            ya1[i+1] = eul[2] + ya1[i]
            """
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R_opt[i]))
            r1[i] = -eul[0]
            p1[i] = -eul[1]
            ya1[i] = -eul[2]
        
        l_sfm = np.zeros(len(self.time)-1)
        L_sfm = 0
        distance = []

        for n in range(len(self.time)-1):
            l_sfm[n] = np.sqrt(
                (t_opt.T[0][n + 1] - t_opt.T[0][n]) ** 2 + (t_opt.T[1][n + 1] - t_opt.T[1][n]) ** 2 + (t_opt.T[2][n + 1] - t_opt.T[2][n]) ** 2)
            L_sfm = L_sfm + l_sfm[n]
            distance.append(L_sfm)
        k_sfm = self.groundtruth[0] / L_sfm
        return np.array(xyz_opt)[:, 0], np.array(xyz_opt)[:, 1], np.array(xyz_opt)[:, 2],r1, p1,ya1, np.array(distance)*k_sfm
    
    
    
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

