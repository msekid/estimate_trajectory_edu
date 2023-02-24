import numpy as np
from init import Init
import matplotlib.pyplot as plt
from calc_traj import CalcTraj
from scipy import integrate, signal, interpolate
import math

class CalcRT():
    def __init__(self):
        self.time = Init().Nx
        self.t_orb = Init().L0[:, 0]
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.L0 = Init().L0
        self.L1 = Init().L0
        self.json_file0 = Init().json_file0
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        

    def calcRT_sfm2(self):
        t = np.array(self.opensfm[5])
        R = np.array(self.opensfm[6])
        RT = []
        hoge = np.array([0, 0, -1.0])
        for i in range(len(t)-1):
            v = t[i+1] - np.dot(R[i+1], np.dot(R[i].T, t[i])) + hoge*1e-3
            RT.append(np.linalg.norm(v/(np.linalg.norm(v))-hoge))
        #print("sfm", np.array(RT).T)
        return np.array(RT).T
    
    def calcRT_sfm(self):
        t = np.array(self.opensfm[5])
        R = np.array(self.opensfm[6])
        RT = []
        v_sum = []
        for i in range(len(t)-1):
            v = t[i+1] - np.dot(R[i+1], np.dot(R[i].T, t[i]))
            #v_norm = v/np.linalg.norm(v)
            v_sum.append(v)
        v_mean = np.average(v_sum, axis=0)
        #print("pre", v_mean)
        v_mean = v_mean/np.linalg.norm(v_mean)
        #print("norm", v_mean)
        v_sfm = []
        for i in range(len(t)-1):
            v = t[i+1] - np.dot(R[i+1], np.dot(R[i].T, t[i]))
            if (v[0] == 0 and v[1] == 0 and v[2] == 0):
                v_norm = 0
            else:
                v_norm = v/np.linalg.norm(v)
            v_sfm.append(v)
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("sfm", np.array(RT).T)
        return np.array(RT).T, v_sfm, R, t
    
    def calcRT_orbslam(self, L):
        x_orb = L[:, 3]
        y_orb = L[:, 1]
        z_orb = L[:, 2]
        q0 = L[:, 7]
        q1 = L[:, 6]
        q2 = L[:, 4]
        q3 = L[:, 5]

        t = np.vstack([x_orb, y_orb, z_orb]).T
        R = []
        for i in range(len(L)):
            R.append([[q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2, 2 * (q1[i] * q2[i] - q0[i] * q3[i]),
                    2 * (q0[i] * q2[i] + q1[i] * q3[i])],
                    [2 * (q0[i] * q3[i] + q1[i] * q2[i]), q0[i] ** 2 - q1[i] ** 2 + q2[i] ** 2 - q3[i] ** 2,
                    2 * (-q0[i] * q1[i] + q2[i] * q3[i])],
                    [2 * (q1[i] * q3[i] - q0[i] * q2[i]), 2 * (q2[i] * q3[i] + q0[i] * q1[i]),
                    q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2]])

        RT = []
        v_sum = []
        v_sum = []
        for i in range(len(t)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])
            #v_norm = v/np.linalg.norm(v)
            v_sum.append(v)
        v_mean = np.average(v_sum, axis=0)
        #print("pre", v_mean)
        v_mean = v_mean/np.linalg.norm(v_mean)
        #print("norm", v_mean)
        
        v_slam = []
        for i in range(len(t)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])
            v_norm = v/np.linalg.norm(v)
            v_slam.append(v)
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("slam", np.array(RT).T)
        return np.array(RT).T, v_slam, R, t
    

    def calcRT_orbslam_interpolated(self):
        time = self.time
        t_orb = self.t_orb
        x_orb = interpolate.interp1d(t_orb, self.L0[:, 3], kind="quadratic",fill_value="extrapolate")(time)
        y_orb = interpolate.interp1d(t_orb, self.L0[:, 1], kind="quadratic",fill_value="extrapolate")(time)
        z_orb = interpolate.interp1d(t_orb, self.L0[:, 2], kind="quadratic",fill_value="extrapolate")(time)
        q0 = interpolate.interp1d(t_orb, self.L0[:, 7], kind="quadratic",fill_value="extrapolate")(time)#7##5
        q1 = interpolate.interp1d(t_orb, self.L0[:, 6], kind="quadratic",fill_value="extrapolate")(time)#6##6
        q2 = interpolate.interp1d(t_orb, self.L0[:, 4], kind="quadratic",fill_value="extrapolate")(time)#4##7
        q3 = interpolate.interp1d(t_orb, self.L0[:, 5], kind="quadratic",fill_value="extrapolate")(time)#5##4

        t = self.orbslam[10]*np.vstack([x_orb, y_orb, z_orb]).T
        r1 = np.zeros(len(q0))
        p1 = np.zeros(len(q0))
        ya1 = np.zeros(len(q0))
        for i in range(len(q0)):
            r1[i] = -(np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
            ya1[i] = -(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i]))+np.pi)
            p1[i] = -(np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))
        
        fig, angle = plt.subplots(figsize=(32, 8))
        time = CalcTraj().Nx
        angle.plot(CalcTraj().time_groundtruth, self.groundtruth[2], color="black", lw=0.5, label="roll_groundtruth")
        angle.plot(CalcTraj().time_groundtruth, self.groundtruth[3], linestyle="dashed",color="black", lw=0.5, label="pitch_groundtruth")
        #angle.plot(time, np.rad2deg(ya1), color="green", lw=0.5, label="yaw")
        angle.plot(time, np.rad2deg(p1), color="red", lw=0.5, label="pitch")
        angle.plot(time, np.rad2deg(r1), color="blue", lw=0.5, label="roll")
        plt.savefig('output/opted/angle.png')
        R = []
        R_ = []
        R_x = []
        R_y = []
        R_z = []
        for i in range(len(q0)):
            R_x.append([[1,                0,                 0], 
                        [0,                math.cos(r1[i]),   -math.sin(r1[i])],
                        [0,                math.sin(r1[1]),   math.cos(r1[i])]])
            R_y.append([[math.cos(p1[i]),  0,                 math.sin(p1[i])], 
                        [0,                1,                 0],
                        [-math.sin(p1[i]), 0,                 math.cos(p1[i])]])
            R_z.append([[math.cos(ya1[i]), -math.sin(ya1[i]), 0], 
                        [math.sin(ya1[i]), math.cos(ya1[i]),  0],
                        [0,                0,                 1]])
            R.append((np.array(R_z[i]) @ np.array(R_y[i]) @ np.array(R_x[i])))

            R_.append([[math.cos(p1[i])*math.cos(r1[i]), math.sin(ya1[i])*math.sin(p1[i])*math.cos(r1[i]) - math.cos(ya1[i])*math.sin(r1[i]), math.sin(ya1[i])*math.sin(r1[i]) + math.cos(ya1[i])*math.sin(p1[i])*math.cos(r1[i])],
                       [math.cos(p1[i])*math.sin(r1[i]), math.sin(ya1[i])*math.sin(p1[i])*math.sin(r1[i]) + math.cos(ya1[i])*math.cos(r1[i]), -math.sin(ya1[i])*math.cos(r1[i]) + math.cos(ya1[i])*math.sin(p1[i])*math.sin(r1[i])], 
                       [-math.sin(p1[i]),                math.sin(ya1[i])*math.cos(p1[i]),                                                    math.cos(ya1[i])*math.cos(p1[i])]])

        '''
        for i in range(len(time)):
            R.append([[q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2, 2 * (q1[i] * q2[i] - q0[i] * q3[i]),
                    2 * (q0[i] * q2[i] + q1[i] * q3[i])],
                    [-2 * (q0[i] * q3[i] + q1[i] * q2[i]), q0[i] ** 2 - q1[i] ** 2 + q2[i] ** 2 - q3[i] ** 2,
                    2 * (-q0[i] * q1[i] + q2[i] * q3[i])],
                    [2 * (q1[i] * q3[i] - q0[i] * q2[i]), 2 * (q2[i] * q3[i] + q0[i] * q1[i]),
                    q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2]])
        '''
        RT = []
        v_sum = []
        for i in range(len(time)-1):
            v = np.dot(np.array(R_[i+1]).T, t[i+1] - t[i])
            #v_norm = v/np.linalg.norm(v)
            v_sum.append(v)
        v_mean = np.average(v_sum, axis=0)
        #print("pre", v_mean)
        v_mean = v_mean/np.linalg.norm(v_mean)
        #print("norm", v_mean)
        
        v_slam = []
        for i in range(len(time)-1):
            v = np.dot(np.array(R_[i+1]).T, t[i+1] - t[i])
            v_norm = v/np.linalg.norm(v)
            v_slam.append(v)
            #print("v_slam", i , np.linalg.norm(v))
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("slam", np.array(RT).T)
        return v_slam, R_, t

    def calc_RT_only(self):
        time = self.time
        t_orb = self.t_orb
        x_orb = interpolate.interp1d(t_orb, self.L1[:, 3], kind="quadratic",fill_value="extrapolate")(time)
        y_orb = interpolate.interp1d(t_orb, self.L1[:, 1], kind="quadratic",fill_value="extrapolate")(time)
        z_orb = interpolate.interp1d(t_orb, self.L1[:, 2], kind="quadratic",fill_value="extrapolate")(time)
        q0 = interpolate.interp1d(t_orb, self.L1[:, 7], kind="quadratic",fill_value="extrapolate")(time)#7##5
        q1 = interpolate.interp1d(t_orb, self.L1[:, 6], kind="quadratic",fill_value="extrapolate")(time)#6##6
        q2 = interpolate.interp1d(t_orb, self.L1[:, 4], kind="quadratic",fill_value="extrapolate")(time)#4##7
        q3 = interpolate.interp1d(t_orb, self.L1[:, 5], kind="quadratic",fill_value="extrapolate")(time)#5##4

        t = self.orbslam[10]*np.vstack([x_orb, y_orb, z_orb]).T
        R = []
        for i in range(len(time)):
            R.append([[q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2, 2 * (q1[i] * q2[i] - q0[i] * q3[i]),
                    2 * (q0[i] * q2[i] + q1[i] * q3[i])],
                    [2 * (q0[i] * q3[i] + q1[i] * q2[i]), q0[i] ** 2 - q1[i] ** 2 + q2[i] ** 2 - q3[i] ** 2,
                    2 * (-q0[i] * q1[i] + q2[i] * q3[i])],
                    [2 * (q1[i] * q3[i] - q0[i] * q2[i]), 2 * (q2[i] * q3[i] + q0[i] * q1[i]),
                    q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2]])

        RT = []
        v_sum = []
        for i in range(len(time)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])
            #v_norm = v/np.linalg.norm(v)
            v_sum.append(v)
        v_mean = np.average(v_sum, axis=0)
        #print("pre", v_mean)
        v_mean = v_mean/np.linalg.norm(v_mean)
        #print("norm", v_mean)
        
        v_slam = []
        for i in range(len(t)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])
            v_norm = v/np.linalg.norm(v)
            v_slam.append(v)
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("slam", np.array(RT).T)
        return np.array(RT).T, v_slam, R

    def showRT(self):
        time = Init().Nx
        fig, rt = plt.subplots(figsize=(32, 8))
        RT_sfm = CalcRT().calcRT_sfm()[0]
        RT_orbslam = CalcRT().calc_RT_only()[0]
        rt.plot(time[1:], np.array(RT_sfm).T, color="red", lw=1.0, label="Normalized vehicle attitude from OpenSfM")
        rt.plot(time[1:], np.array(RT_orbslam).T, color="green", lw=1.0, label="Normalized vehicle attitude from ORB-SLAM2")
#        rt.plot(self.orbslam[6][1:], np.array(RT_orbslam).T, color="green", lw=1.0, label="Normalized vehicle attitude from ORB-SLAM2")
        rt.legend(fancybox=False, shadow=False, edgecolor='black')
        rt.set_xlabel("Time [s]")
        rt.set_ylabel("Normalized vehicle attitude")
        plt.grid(True)
        plt.ylim(0, 0.6)
        plt.savefig('output/opted/RT.png')
        #plt.show()