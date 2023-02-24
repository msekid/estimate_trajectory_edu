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
MakeTestList().exportFile()


class CalcTraj():
    def __init__(self):
        self.n_frame = Init().n_frame
        self.frame = self.n_frame - 3
        self.len_groundtruth = Init().len_groundtruth
        self.time_groundtruth = Init().time_groundtruth
        self.Nx = Init().Nx

    def calcGroundTruth(self, N, M):
        time = self.time_groundtruth[:]
        vel_km = N[:]
        vel = vel_km / 3.6
        ac_x = M[:, 0]
        ac_y = M[:, 1]
        ac_z = M[:, 2]
        FPS = 14  #ここを変更
        dt = 1 / FPS

        # true of posture姿勢の真値
        #yaw_t = M[:, 0] - 360
        #for j in range(len(yaw_t)):
        #    if (yaw_t[j] < -300):
        #        yaw_t[j] = yaw_t[j] + 360
        #pitch_t = M[:, 1]
        #roll_t = M[:, 2]
        #yawrate = M[:, 3]

        # 速度（車速パルス）→移動距離
        distance = np.trapz(vel, None, dt)
        cdistance = integrate.cumtrapz(vel, None, dt)
        T = np.dstack([time, np.insert(cdistance, 0, 0)])

        # ac2vel
        # 加速度→速度→速さ
        cvelo_x1 = integrate.cumtrapz(ac_x, None, dt)
        cvelo_x = np.zeros(len(cvelo_x1))
        for i in range(len(cvelo_x)):
            cvelo_x[i] = cvelo_x1[i] + vel[0]
        cvelo_y = integrate.cumtrapz(ac_y, None, dt)
        speed = np.sqrt(cvelo_x ** 2 + cvelo_y ** 2)

        # vel2dis
        # 速さ→移動距離
        distance_xy = np.trapz(speed, None, dt)
        cdistance_xy = integrate.cumtrapz(speed, None, dt)
        T_xy = np.dstack([time[0:self.len_groundtruth-1], np.insert(cdistance_xy, 0, 0)])
        # ロール，ピッチ角（加速度から）
        r = np.rad2deg(np.arctan(ac_y / ac_z))
        p = np.rad2deg(np.arctan(- ac_x / np.sqrt(ac_y ** 2 + ac_z ** 2)))
        L_v = cdistance[self.len_groundtruth-2]

        #theta = np.zeros(self.len_groundtruth)
        #x_t = np.zeros(self.len_groundtruth)
        #y_t = np.zeros(self.len_groundtruth)
        #for i in range(self.len_groundtruth - 1):
        #    theta[i + 1] = theta[i] + np.deg2rad((yawrate[i] * dt))
        #    x_t[i + 1] = x_t[i] + vel[i] * np.cos(theta[i]) * dt
        #    y_t[i + 1] = y_t[i] + vel[i] * np.sin(theta[i]) * dt

        #l_t = np.zeros(self.len_groundtruth)
        #L_t = 0
        #for i in range(self.frame - 1):
        #    l_t[i] = np.sqrt((x_t[i + 1] - x_t[i]) ** 2 + (y_t[i + 1] - y_t[i]) ** 2)
        #    L_t = L_t + l_t[i]
        
        return L_v, ac_x, ac_y, ac_z

    def calcOrbslam(self, groundtruth, L):
        time = self.Nx
        t_orb = L[:, 0]  # /1000000000
        x_orb = L[:, 3]
        y_orb = L[:, 1]
        z_orb = -L[:, 2]
        q0 = L[:, 4]#7
        q1 = L[:, 5]#6
        q2 = L[:, 6]#4
        q3 = L[:, 7]#5
        r1 = np.zeros(len(q0))
        p1 = np.zeros(len(q0))
        ya1 = np.zeros(len(q0))
        for i in range(len(q0)):
            r1[i] = np.rad2deg(
                np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
            p1[i] = np.rad2deg(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i])))
            ya1[i] = np.rad2deg(
                np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))
        f1 = interpolate.interp1d(t_orb, r1, kind="linear", fill_value=(r1[0], r1[len(r1)-1]),bounds_error=False)#(r1[0], r1[len(r1)-1])
        f2 = interpolate.interp1d(t_orb, ya1, kind="linear", fill_value=(ya1[0], ya1[len(ya1)-1]),bounds_error=False)
        f3 = interpolate.interp1d(t_orb, p1, kind="linear", fill_value=(p1[0], p1[len(p1)-1]),bounds_error=False)
        f4 = interpolate.interp1d(t_orb, x_orb, kind="linear", fill_value="extrapolate")
        f5 = interpolate.interp1d(t_orb, y_orb, kind="linear", fill_value="extrapolate")
        f6 = interpolate.interp1d(t_orb, z_orb, kind="linear", fill_value="extrapolate")
        r1_re = f1(time)
        ya1_re = -f3(time)
        p1_re = f2(time)
        
        x_re = f4(time)
        y_re = f5(time)
        z_re = f6(time)
        l_orb = np.zeros(len(time))
        L_orb = 0
        distance = []
        for n in range(len(x_re)-1):
            l_orb[n] = np.sqrt((x_re[n + 1] - x_re[n]) ** 2 + (y_re[n + 1] - y_re[n]) ** 2 + (z_re[n + 1] - z_re[n]) ** 2)
            L_orb = L_orb + l_orb[n]
            distance.append(L_orb)

        k_v=groundtruth[0]/L_orb
        x_vf = x_re * k_v
        y_vf = y_re * k_v
        z_vf = z_re * k_v
        return y_vf, x_vf, z_vf, r1_re, p1_re, ya1_re, t_orb, x_re, y_re, z_re, k_v, np.array(distance)*k_v, y_vf-y_vf[0], x_vf-x_vf[0], z_vf-z_vf[0]

    @staticmethod
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

        return np.array([-roll, -pitch, yaw])
    
    def calcOpensfm(self, groundtruth, json_file):
        time = self.Nx
        json_object = json.load(json_file)
        f = open('test_list.txt', 'r')
        name_data = f.read().splitlines()
        f.close()

        roll_est = np.zeros(len(name_data))
        pitch_est = np.zeros(len(name_data))
        yaw_est = np.zeros(len(name_data))
        x_est = np.zeros(len(name_data))
        y_est = np.zeros(len(name_data))
        z_est = np.zeros(len(name_data))
        list = np.array(range(len(name_data)))

        for i, j in zip(list, name_data):
            try:
                roll_est[i] = json_object[0]["shots"][j]["rotation"][0]
                pitch_est[i] = json_object[0]["shots"][j]["rotation"][1]
                yaw_est[i] = json_object[0]["shots"][j]["rotation"][2]
                x_est[i] = json_object[0]["shots"][j]["translation"][0]
                y_est[i] = json_object[0]["shots"][j]["translation"][1]
                z_est[i] = json_object[0]["shots"][j]["translation"][2]
            except KeyError:
                #print("エラーです", i, j)
                continue
        #print(roll_est[0], pitch_est[0], yaw_est[0])
        t = np.vstack([x_est, y_est, z_est]).T
        R3 = []
        for i in range(len(name_data)):
            R3.append(np.array(cv2.Rodrigues(np.array([roll_est[i], pitch_est[i], yaw_est[i]]))[0]).T)
        xyz = []
        for i in range(len(name_data)):
            xyz.append(-np.dot(np.array(R3[i]).T.T, t[i]))
        x_sfm = np.array(xyz)[:, 0]
        y_sfm = np.array(xyz)[:, 1]
        z_sfm = np.array(xyz)[:, 2]
        l_sfm = np.zeros(len(name_data)-1)
        L_sfm = 0
        distance = []


        for n in range(len(name_data)-1):
            l_sfm[n] = np.sqrt(
                (x_sfm[n + 1] - x_sfm[n]) ** 2 + (y_sfm[n + 1] - y_sfm[n]) ** 2 + (z_sfm[n + 1] - z_sfm[n]) ** 2)
            L_sfm = L_sfm + l_sfm[n]
            distance.append(L_sfm)
        k_sfm = groundtruth[0] / L_sfm
        #y_ = np.vstack([groundtruth[2], groundtruth[3], groundtruth[4]]).T
        x_ = k_sfm*np.vstack([x_sfm[:], y_sfm[:], z_sfm[:]]).T  # colmap

        #x_ = x_ - x_.mean(axis=0)  # genten
        #y_ = y_ - y_.mean(axis=0)
        #U, S, V = np.linalg.svd(x_.T @ y_)
        #U, S, V = np.linalg.svd(np.diff(np.diff(x_, axis=0), axis=0).T @ y_)
        #R = V.T @ U.T
        #x_ = (R @ x_.T).T
        r1 = np.zeros(len(name_data))
        p1 = np.zeros(len(name_data))
        ya1 = np.zeros(len(name_data))
        #角度の計算

        #R4 = R3 @ np.linalg.inv(R3)
        for i in range(len(name_data)):
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R3[i]))
            r1[i] = -eul[0]
            p1[i] = eul[1] - 180
            ya1[i] = -eul[2]
        R_ = []
        return x_[:, 0], x_[:, 1], r1, p1+90,ya1, t*k_sfm, R3,k_sfm*np.array(xyz), R_, x_[:, 2], np.array(distance)*k_sfm, x_[:, 0]-x_[:, 0][0], x_[:, 1]-x_[:, 1][0], x_[:, 2]-x_[:, 2][0]

    
    def calcDroidslam(self, groundtruth, L):
        time = self.Nx
        #quotient = len(groundtruth[0]) // len(time)
        #remainder = len(groundtruth[0]) % len(time)
        t_orb = L[:, 0]
        '''
        q0 = interpolate.interp1d(t_orb, L[:, 4], kind="quadratic",fill_value="extrapolate")(time)#7##5
        q1 = interpolate.interp1d(t_orb, L[:, 7], kind="quadratic",fill_value="extrapolate")(time)#6##6
        q2 = interpolate.interp1d(t_orb, L[:, 6], kind="quadratic",fill_value="extrapolate")(time)#4##7
        q3 = interpolate.interp1d(t_orb, L[:, 5], kind="quadratic",fill_value="extrapolate")(time)#5##4
        '''

        t_x = L[:, 1]
        t_y = L[:, 2]
        t_z = L[:, 3]
        
        q0 = L[:, 4]
        q1 = L[:, 7]
        q2 = L[:, 6]
        q3 = L[:, 5]

        R = []
        for i in range(len(L)):
            R.append([[q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2, 2 * (q1[i] * q2[i] - q0[i] * q3[i]),
                    2 * (q0[i] * q2[i] + q1[i] * q3[i])],
                    [2 * (q0[i] * q3[i] + q1[i] * q2[i]), q0[i] ** 2 - q1[i] ** 2 + q2[i] ** 2 - q3[i] ** 2,
                    2 * (-q0[i] * q1[i] + q2[i] * q3[i])],
                    [2 * (q1[i] * q3[i] - q0[i] * q2[i]), 2 * (q2[i] * q3[i] + q0[i] * q1[i]),
                    q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2]])
        R3 = R.copy()
        t = np.vstack([t_z, t_y, t_x]).T
        #t = np.vstack([t_z, t_y, t_x]).T
        xyz = []
        for i in range(len(L)):
            xyz.append(-np.dot(np.array(R[i]).T, t[i]))
        xyz = np.array(xyz)
        x_droid = interpolate.interp1d(t_orb*(1/14), xyz[:, 0], kind="linear",fill_value="extrapolate")(time)#7##5
        y_droid = interpolate.interp1d(t_orb*(1/14), xyz[:, 1], kind="linear",fill_value="extrapolate")(time)#7##5
        z_droid = interpolate.interp1d(t_orb*(1/14), xyz[:, 2], kind="linear",fill_value="extrapolate")(time)#7##5
        '''
        x_droid = xyz[:, 0]
        y_droid = xyz[:, 1]
        z_droid = xyz[:, 2]
        '''
        t_ = np.vstack([z_droid, x_droid, y_droid]).T
        t__ = np.vstack([x_droid, y_droid, z_droid]).T
        l_sfm = np.zeros(len(time)-1)
        L_sfm = 0
        distance = []

        for n in range(len(time)-1):
            l_sfm[n] = np.sqrt(
                (x_droid[n + 1] - x_droid[n]) ** 2 + (y_droid[n + 1] - y_droid[n]) ** 2 + (z_droid[n + 1] - z_droid[n]) ** 2)
            L_sfm = L_sfm + l_sfm[n]
            distance.append(L_sfm)
        k_sfm = groundtruth[0] / L_sfm
        #y_ = np.vstack([groundtruth[0][remainder::quotient], groundtruth[1][remainder::quotient], np.zeros(len(groundtruth[0][remainder::quotient]))]).T
        x_ = k_sfm*np.vstack([z_droid[:], x_droid[:], y_droid[:]]).T  # colmap
        #x_ = k_sfm*np.vstack([z_droid[:], y_droid[:], x_droid[:]]).T
        #x_ = x_ - x_.mean(axis=0)  # genten
        #y_ = y_ - y_.mean(axis=0)
        #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
        #R = V.T @ U.T
        #x_ = (R @ x_.T).T
        #droid_x = interpolate.interp1d(t_orb*(1/14), x_[:, 0], kind="linear",fill_value="extrapolate")(time)#7##5
        #droid_y = interpolate.interp1d(t_orb*(1/14), x_[:, 1], kind="linear",fill_value="extrapolate")(time)#7##5
        q0 = L[:, 4]#7
        q1 = L[:, 5]#6
        q2 = L[:, 6]#4
        q3 = L[:, 7]#5
        r1 = np.zeros(len(q0))
        p1 = np.zeros(len(q0))
        ya1 = np.zeros(len(q0))
        for i in range(len(q0)):
            r1[i] = np.rad2deg(
                np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
            p1[i] = np.rad2deg(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i])))
            ya1[i] = np.rad2deg(
                np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))
        f1 = interpolate.interp1d(t_orb*(1/14), r1, kind="linear", fill_value=(r1[0], r1[len(r1)-1]),bounds_error=False)#(r1[0], r1[len(r1)-1])
        f2 = interpolate.interp1d(t_orb*(1/14), ya1, kind="linear", fill_value=(ya1[0], ya1[len(ya1)-1]),bounds_error=False)
        f3 = interpolate.interp1d(t_orb*(1/14), p1, kind="linear", fill_value=(p1[0], p1[len(p1)-1]),bounds_error=False)
        r1_re = -f1(time)
        ya1_re = f3(time)
        p1_re = -f2(time)
        return -(x_[:, 0]), x_[:, 1], x_[:, 2],r1_re, p1_re, ya1_re, t_*k_sfm, t__*k_sfm, R3, x_, np.array(distance)*k_sfm, -(x_[:, 0]-x_[:, 0][0]), x_[:, 1]-x_[:, 1][0], x_[:, 2]-x_[:, 2][0]

