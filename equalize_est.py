import json
import cv2
import numpy as np
from scipy import integrate, signal, interpolate
import matplotlib.pyplot as plt
import matplotlib
import matplotlib as mpl
import csv
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import math
import glob
from init import Init
from make_testlist import MakeTestList
from calc_traj import CalcTraj
from calc_RT__ import CalcRT

class Equalize():
    def __init__(self):
        self.n_frame = Init().n_frame
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)



    def euler2RotationMatrix(self, roll, pitch, yaw):
        R_ = []
        R_x = []
        R_y = []
        R_z = []
        for i in range(self.n_frame):
            p_ = (np.pi+roll[i])
            r_ = pitch[i]
            ya_ = -(np.pi+yaw[i])
            R_x.append([[1,                0,                 0], 
                        [0,                math.cos(r_),   -math.sin(r_)],
                        [0,                math.sin(r_),   math.cos(r_)]])

            R_y.append([[math.cos(p_),  0,                 math.sin(p_)], 
                        [0,                1,                 0],
                        [-math.sin(p_), 0,                 math.cos(p_)]])

            R_z.append([[math.cos(ya_), -math.sin(ya_), 0], 
                        [math.sin(ya_), math.cos(ya_),  0],
                        [0,                0,                 1]])
            R_.append((np.array(R_z[i]) @ np.dot(np.array(R_y[i]), np.array(R_x[i]))))
        
        return R_

    
    def rotationMatrixToEulerAngles(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            pitch = math.atan2(R[2, 1], R[2, 2])
            roll = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            pitch = math.atan2(-R[1, 2], R[1, 1])
            roll = math.atan2(-R[2, 0], sy)
            yaw = 0

        return np.array([roll, pitch, yaw])


    def calcQ(self, R):
        Q = []
        for i in range(self.n_frame):
            if (i == 0):
                Q.append(np.array(R[i]))
            else:
                Q.append(R[i] @ np.linalg.inv(R[i-1]))
        return Q
    
    def calcV(self, t, R):
        v = []
        for i in range(len(self.time)):
            if (i == 0):
                v.append([0, 0, 0])
            else:
                v.append(np.dot(np.array(R[i]).T, t[i] - t[i-1]))
        return v

    def averagedSLAM(self, a, b, c, d, e, f, g, h, i, j, slam):
        L = []
        for data in [a, b, c, d, e, f, g, h, i, j]:
            if(len(data) == 0):
                continue
            elif (slam == 'ORBSLAM' and data[0][0] != 0):
                continue
            else:
                L.append(data.tolist())
        output_all = []
        x = []
        y = []
        z = []
        roll = []
        pitch = []
        yaw = []
        #print(len(Init().L) == 0)
        for i in range(len(L)):
            if (slam == 'DROIDSLAM'):
                orbslam = CalcTraj().calcDroidslam(self.groundtruth, np.array(L[i]))
            elif (len(Init().L) == 0):
                orbslam = CalcTraj().calcDroidslam(self.groundtruth, np.array(L[i]))
            else:
                orbslam = CalcTraj().calcOrbslam(self.groundtruth, np.array(L[i]))
            x.append(orbslam[0])
            y.append(orbslam[1])
            z.append(orbslam[2])
            roll.append(orbslam[3])
            pitch.append(orbslam[4])
            yaw.append(orbslam[5])
        
        
        x_ave = []
        y_ave = []
        z_ave = []
        roll_ave = []
        pitch_ave = []
        yaw_ave = []
        for i in range(self.n_frame):
            x_ave.append(np.median(np.array(x).T[i]))
            y_ave.append(np.median(np.array(y).T[i]))
            z_ave.append(np.median(np.array(z).T[i]))
            roll_ave.append(np.median(np.array(roll).T[i]))
            pitch_ave.append(np.median(np.array(pitch).T[i]))
            yaw_ave.append(np.median(np.array(yaw).T[i]))
        
        return x_ave, y_ave, z_ave, roll_ave, pitch_ave, yaw_ave


    def equalizeORBSLAM(self):
        L = []
        for data in [self.L0, self.L1, self.L2, self.L3, self.L4, self.L5, self.L6, self.L7, self.L8, self.L9]:
            if(len(data) == 0):
                continue
            else:
                L.append(data.tolist())
        output_all = []
        t = []
        v = []
        R = []
        for i in range(len(L)):
            orbslam = CalcTraj().calcOrbslam(self.groundtruth, np.array(L[i]))
            output = CalcRT().calcRT(orbslam[0], orbslam[1], orbslam[2], orbslam[3], orbslam[4], orbslam[5])
            v.append(np.array(output[1]))
            t.append(np.array(output[0]))
            R.append(np.array(output[2]))
        #for i in range(len(np.array(output_all).T)):
        #    for j in range(len(np.array(output_all).T[0])):
            
        #        print(len(np.array(output_all).T[0]))
        #print(np.array(t[0]))
        #R = []
        Q = []
        for i in range(len(L)):
            #R_ = self.euler2RotationMatrix(np.array(output_all)[i].T[3], np.array(output_all)[i].T[4], np.array(output_all)[i].T[5])
            #R.append(R_)
            Q.append(self.calcQ(R[i]))
        
        euler_all = []
        for i in range(len(L)):
            euler =  []
            r1 = np.zeros(self.n_frame)
            p1 = np.zeros(self.n_frame)
            ya1 = np.zeros(self.n_frame)
            for j in range(self.n_frame):
                eul = np.rad2deg(self.rotationMatrixToEulerAngles(Q[i][j]))
                r1[j] = -eul[0]
                p1[j] = eul[1] % 360 - 180
                ya1[j] = -eul[2]
                #euler.append(np.array(self.rotationMatrixToEulerAngles(Q[i][j])))
            euler_all.append(np.vstack([r1, p1, ya1]).T.tolist())
            #print(np.array(euler_all)[0])
        euler_equalized = []
        for i in range(self.n_frame):
            euler_equalized.append([np.median(np.array(euler_all).T[0][i]), np.median(np.array(euler_all).T[1][i]), np.median(np.array(euler_all).T[2][i])])

        Q_equalized = self.euler2RotationMatrix(np.deg2rad(np.array(euler_equalized).T[0]), np.deg2rad(np.array(euler_equalized).T[1]), np.deg2rad(np.array(euler_equalized).T[2]))
        
        '''
        v = []
        for i in range(len(L)):
            for j in range(self.n_frame - 1):
                if (j == 0):
                    v.append([0, 0, 0])
                else:
                    #print(t[i][j])
                    v_ = np.dot(np.array(R[i][j]).T, (np.array(t)[i][j]-np.array(t)[i][j-1]))
                    v.append(v_)
        '''
        v_equalized = []
        for i in range(self.n_frame):
            v_equalized.append([np.median(np.array(v).T[0][i]), np.median(np.array(v).T[1][i]), np.median(np.array(v).T[2][i])])
        
        #print(Q_equalized)
        #print(v_equalized)
        return Q_equalized, v_equalized


    def show(self):
        R_r = np.zeros((self.n_frame, 3, 3))
        t = np.zeros((self.n_frame, 3))
        Q = self.equalizeORBSLAM()[0]
        v = self.equalizeORBSLAM()[1]
        for i in range(self.n_frame):
            if (i == 0):
                R_r[i] = np.array(Q[i])
                t[i] = np.array(v[i])
            else:
                R_r[i] = np.array(Q[i]) @ np.array(R_r[i-1])
                t[i] = np.array(t[i-1]) + np.array(R_r[i]) @ np.array(v[i])
        r1 = np.zeros(self.n_frame)
        p1 = np.zeros(self.n_frame)
        ya1 = np.zeros(self.n_frame)

        for i in range(self.n_frame):
            eul = np.rad2deg(self.rotationMatrixToEulerAngles(R_r[i]))
            r1[i] = -eul[0]
            p1[i] = eul[1] % 360 - 180
            ya1[i] = -eul[2]

        fig, traj = plt.subplots()
        traj.plot(np.array(t).T[0], np.array(t).T[1])
        traj.set_aspect('equal')
        plt.show()
        time = CalcTraj().Nx
        fig, roll = plt.subplots()
        #print(r1)
        roll.plot(time, r1)
        plt.show()
        fig, pitch = plt.subplots()
        pitch.plot(time, p1)
        plt.show()
        fig, yaw = plt.subplots()
        yaw.plot(time, ya1)
        plt.show()
        
    def show2(self):
        x = self.averagedORBSLAM()[0]
        y = self.averagedORBSLAM()[1]
        z = self.averagedORBSLAM()[2]
        roll = self.averagedORBSLAM()[3]
        #print(np.array(roll).shape)
        time = CalcTraj().Nx
        #print(np.array(time).shape)
        pitch = self.averagedORBSLAM()[4]
        yaw = self.averagedORBSLAM()[5]
        fig, traj = plt.subplots()
        traj.plot(x, y)
        traj.set_aspect('equal')
        plt.show()
        fig, roll1 = plt.subplots()
        #print(r1)
        roll1.plot(np.array(time), np.array(roll))
        plt.show()
        fig, pitch1 = plt.subplots()
        pitch1.plot(np.array(time), np.array(pitch))
        plt.show()
        fig, yaw1 = plt.subplots()
        yaw1.plot(np.array(time), np.array(yaw))
        plt.show()
            
#a = Equalize()
#a.show2()