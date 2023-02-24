import json
import cv2
import numpy as np
from scipy import integrate, signal, interpolate
import matplotlib.pyplot as plt
import matplotlib
plt.rcParams['font.family'] = 'Times New Roman' #全体のフォントを設定
import matplotlib as mpl
import csv
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
#import pandas as pd
import math

frame = 224-3
Nx = np.loadtxt('/home/sora-desktop/dataset/sensor.csv', delimiter=',', skiprows=1, usecols=[2, 3, 4, 5, 6, 7, 8])
N2x = np.loadtxt('/home/sora-desktop/dataset/drive-download-20220307T065449Z-001/denso_20201125_turn_left4/times_28.txt')
def calcGroundTruth(N, M):

    time = N[:, 0]
    vel_km = N[:, 1]
    vel = vel_km / 3.6
    lat = N[:, 2]
    lon = N[:, 3]
    ac_x = N[:, 4]
    ac_y = N[:, 5]
    ac_z = N[:, 6]
    FPS = 14
    dt = 1 / FPS

    # true of posture姿勢の真値
    yaw_t = M[:, 15] - 360
    for j in range(len(yaw_t)):
        if (yaw_t[j] < -300):
            yaw_t[j] = yaw_t[j] + 360
    pitch_t = M[:, 16]
    roll_t = M[:, 17]
    yawrate = M[:, 20]

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
    T_xy = np.dstack([time[0:223], np.insert(cdistance_xy, 0, 0)])
    # ロール，ピッチ角（加速度から）
    r = np.rad2deg(np.arctan(ac_y / ac_z))
    p = np.rad2deg(np.arctan(- ac_x / np.sqrt(ac_y ** 2 + ac_z ** 2)))

    theta = np.zeros(frame)
    x_t = np.zeros(frame)
    y_t = np.zeros(frame)
    for i in range(frame - 1):
        theta[i + 1] = theta[i] + np.deg2rad((yawrate[i] * dt))
        x_t[i + 1] = x_t[i] + vel[i] * np.cos(theta[i]) * dt
        y_t[i + 1] = y_t[i] + vel[i] * np.sin(theta[i]) * dt

    l_t = np.zeros(frame)
    L_t = 0
    for i in range(frame - 1):
        l_t[i] = np.sqrt((x_t[i + 1] - x_t[i]) ** 2 + (y_t[i + 1] - y_t[i]) ** 2)
        L_t = L_t + l_t[i]
    L_v = cdistance[frame]
    return x_t-x_t[0], y_t-y_t[0], roll_t-roll_t[0], pitch_t-pitch_t[0], yaw_t-yaw_t[0], L_t, L_v

def calcOrbslam(groundtruth, L):
    time = Nx[:, 0]
    t_orb = L[:, 0]  # /1000000000
    x_orb = L[:, 3]
    y_orb = L[:, 1]
    z_orb = L[:, 2]
   
    q0 = L[:, 7]
    q1 = L[:, 6]
    q2 = L[:, 4]
    q3 = L[:, 5]
    r1 = np.zeros(len(q0))
    p1 = np.zeros(len(q0))
    ya1 = np.zeros(len(q0))
    for i in range(len(q0)):
        r1[i] = np.rad2deg(
            np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
        p1[i] = np.rad2deg(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i])))
        ya1[i] = np.rad2deg(
            np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))
    f1 = interpolate.interp1d(t_orb, r1, kind="linear", fill_value="extrapolate")
    f2 = interpolate.interp1d(t_orb, ya1, kind="linear", fill_value="extrapolate")
    f3 = interpolate.interp1d(t_orb, p1, kind="linear", fill_value="extrapolate")
    f4 = interpolate.interp1d(t_orb, x_orb, kind="linear", fill_value="extrapolate")
    f5 = interpolate.interp1d(t_orb, y_orb, kind="linear", fill_value="extrapolate")
    f6 = interpolate.interp1d(t_orb, z_orb, kind="linear", fill_value="extrapolate")
    ya1_re = f2(time)
    p1_re = f3(time)
    r1_re = f1(time)
    x_re = f4(time)
    y_re = f5(time)
    z_re = f6(time)

    l_orb = np.zeros(frame)
    L_orb = 0
    for n in range(len(L) - 1):
        l_orb[n] = np.sqrt((x_orb[n + 1] - x_orb[n]) ** 2 + (y_orb[n + 1] - y_orb[n]) ** 2)
        L_orb = L_orb + l_orb[n]

    k_v=groundtruth[6]/L_orb
    x_vf = x_re * k_v
    y_vf = y_re * k_v
    z_vf = z_re * k_v
    return x_vf-x_vf[0], y_vf-y_vf[0], z_vf, r1_re, p1_re, ya1_re


def rotationMatrixToEulerAngles(R) :
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


def calcOpensfm(groundtruth, json_file):
    json_object = json.load(json_file)
    f = open('/home/sora-desktop/OpenSfM/data/denso_20201125_turn_left4/test_list.txt', 'r')
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
        roll_est[i] = json_object[0]["shots"][j]["rotation"][0]
        pitch_est[i] = json_object[0]["shots"][j]["rotation"][1]
        yaw_est[i] = json_object[0]["shots"][j]["rotation"][2]
        x_est[i] = json_object[0]["shots"][j]["translation"][0]
        y_est[i] = json_object[0]["shots"][j]["translation"][1]
        z_est[i] = json_object[0]["shots"][j]["translation"][2]
    t = np.vstack([x_est, y_est, z_est]).T
    R3 = []
    for i in range(len(name_data)):
        R3.append(cv2.Rodrigues(np.array([roll_est[i], pitch_est[i], yaw_est[i]]))[0])
    xyz = []
    for i in range(len(name_data)):
        xyz.append(-np.dot(np.array(R3[i]).T, t[i]))
    x_sfm = np.array(xyz)[:, 0]
    y_sfm = np.array(xyz)[:, 1]
    z_sfm = np.array(xyz)[:, 2]
    l_sfm = np.zeros(len(name_data)-1)
    L_sfm = 0

    for n in range(len(name_data)-1):
        l_sfm[n] = np.sqrt(
            (x_sfm[n + 1] - x_sfm[n]) ** 2 + (y_sfm[n + 1] - y_sfm[n]) ** 2 + (z_sfm[n + 1] - z_sfm[n]) ** 2)
        L_sfm = L_sfm + l_sfm[n]
    k_sfm = groundtruth[6] / L_sfm
    y_ = np.vstack([groundtruth[0], groundtruth[1], np.zeros(len(groundtruth[0]))]).T
    x_ = k_sfm*np.vstack([x_sfm[0:-3], y_sfm[0:-3], z_sfm[0:-3]]).T  # colmap

#    x__ = x_[10:90,:] - x_[10:90,:].mean(axis=0)  # genten
#    y__ = y_[10:90,:] - y_[10:90,:].mean(axis=0)
    
    x_ = x_ - x_.mean(axis=0)  # genten
    y_ = y_ - y_.mean(axis=0)

    U, S, V = np.linalg.svd(x_.T @ y_)

    #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
    R = V.T @ U.T
    x_ = (R @ x_.T).T
    r1 = np.zeros(len(name_data))
    p1 = np.zeros(len(name_data))
    ya1 = np.zeros(len(name_data))
    #角度の計算

    R4 = R @ np.linalg.inv(R3)
#    R4 = R3
    for i in range(len(name_data)):
        R4[i] = R @ np.linalg.inv(R3[i])
#        R4[i] = R @ R3[i]
#        R4[i] = R3[i] @ np.linalg.inv(R)
#        r1[i] = np.rad2deg(np.arctan2(np.array(R4[i]).T[2][1], np.array(R4[i]).T[2][2]))
#        p1[i] = np.rad2deg(-np.arcsin(np.array(R4[i]).T[2][0]))
#        ya1[i] = np.rad2deg(np.arctan2(np.array(R4[i]).T[1][0], np.array(R4[i]).T[0][0]))
        eul = np.rad2deg(rotationMatrixToEulerAngles(R4[i]))
        r1[i] = eul[0]
        p1[i] = eul[1]
        ya1[i] = eul[2]
        #r1[i] = np.rad2deg(np.arctan(np.array(R3[i]).T[2][0]/np.array(R3[i]).T[1][0]))
        #p1[i] = np.rad2deg(np.arctan(-np.array(R3[i]).T[0][2]/np.array(R3[i]).T[0][1]))
        #ya1[i] = np.rad2deg(np.arctan(np.array(R3[i]).T[1][0] / np.array(R3[i]).T[0][0]/np.cos(np.arctan(np.array(R3[i]).T[2][0]/np.array(R3[i]).T[1][0]))))

    return x_[:, 0]-x_[:, 0][0], x_[:, 1]-x_[:, 1][0], r1-r1[0], p1-p1[0],(ya1-ya1[0]), t, R3

def calcColmap(groundtruth, L1):
    L2 = L1[0::2]
    L = L2[np.argsort(L2[:,0])]
    print(L)
    q0 = L[:, 1]
    q1 = L[:, 2]
    q2 = L[:, 3]
    q3 = L[:, 4]
    r1 = np.zeros(len(q0))
    p1 = np.zeros(len(q0))
    ya1 = np.zeros(len(q0))
    R = []
    for i in range(len(L)):
        R.append([[q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2, 2 * (q1[i] * q2[i] - q0[i] * q3[i]),
                   2 * (q0[i] * q2[i] + q1[i] * q3[i])],
                  [2 * (q0[i] * q3[i] + q1[i] * q2[i]), q0[i] ** 2 - q1[i] ** 2 + q2[i] ** 2 - q3[i] ** 2,
                   2 * (-q0[i] * q1[i] + q2[i] * q3[i])],
                  [2 * (q1[i] * q3[i] - q0[i] * q2[i]), 2 * (q2[i] * q3[i] + q0[i] * q1[i]),
                   q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2]])
    q0 = L[:, 4]
    q1 = L[:, 3]
    q2 = L[:, 1]
    q3 = L[:, 2]
    for i in range(len(q0)):
        r1[i] = np.rad2deg(
            np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
        p1[i] = np.rad2deg(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i])))
        ya1[i] = np.rad2deg(
            np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))

    t = L[:, 5:8]
    xyz = []
    for i in range(len(L)):
        xyz.append(-np.dot(np.array(R[i]).T, t[i]))
    l_cmp = np.zeros(len(L)-1)
    L_cmp = 0
    x_cmp = np.array(xyz)[:, 0]
    y_cmp = np.array(xyz)[:, 1]
    z_cmp = np.array(xyz)[:, 2]

    for n in range(len(L)-1):
        l_cmp[n] = np.sqrt(
            (x_cmp[n + 1] - x_cmp[n]) ** 2 + (y_cmp[n + 1] - y_cmp[n]) ** 2 + (z_cmp[n + 1] - z_cmp[n]) ** 2)
        L_cmp = L_cmp + l_cmp[n]

    k_cmp = groundtruth[6] / L_cmp
    y_ = np.vstack([groundtruth[0], groundtruth[1], np.zeros(len(groundtruth[0]))]).T
    x_ = k_cmp*np.vstack([x_cmp[3:], y_cmp[3:], z_cmp[3:]]).T#colmap

    x_ = x_ - x_.mean(axis=0)#genten
    y_ = y_ - y_.mean(axis=0)
    U, S, V = np.linalg.svd(x_.T @ y_)
    #U, S, V = np.linalg.svd(np.diff(x_, axis=0).T @ np.diff(y_, axis=0))
    R1 = V.T @ U.T
    x_ = (R1 @ x_.T).T

    R4 = R1 @ np.linalg.inv(R)
    for i in range(len(L)):
        R4[i] = R1 @ np.linalg.inv(R[i])
#        R4[i] = R1 @ R[i]
        r1[i] = np.rad2deg(np.arctan2(np.array(R4[i]).T[2][1], np.array(R4[i]).T[2][2]))
        p1[i] = np.rad2deg(-np.arcsin(np.array(R4[i]).T[2][0]))
        ya1[i] = np.rad2deg(np.arctan2(np.array(R4[i]).T[1][0], np.array(R4[i]).T[0][0]))
        r1_ = np.rad2deg(
            np.arctan(2 * (q0[i] * q1[i] + q2[i] * q3[i]) / (q0[i] ** 2 - q1[i] ** 2 - q2[i] ** 2 + q3[i] ** 2)))
        p1_ = np.rad2deg(np.arcsin(2 * (q0[i] * q2[i] - q1[i] * q3[i])))
        ya1_ = np.rad2deg(
            np.arctan(2 * (q0[i] * q3[i] + q2[i] * q1[i]) / (q0[i] ** 2 + q1[i] ** 2 - q2[i] ** 2 - q3[i] ** 2)))
        eul = np.rad2deg(rotationMatrixToEulerAngles(R4[i]))
        r1[i] = eul[0]
        p1[i] = eul[1]
        ya1[i] = eul[2]
        #print([r1[i], p1[i], ya1[i], r1_, p1_, ya1_])
        #print(eul)

    return x_[:, 0]-x_[:, 0][0], x_[:, 1]-x_[:, 1][0],r1-r1[0],p1-p1[0],ya1-ya1[0]

def showTrajectory(groundtruth, opensfm):
    fig, traj = plt.subplots()
    traj.plot(groundtruth[1], groundtruth[0], color="black", lw=1.0, label="Ground Truth")
    traj.scatter(opensfm[1][0::29], opensfm[0][0::29], s=10, color="black")
    traj.plot(opensfm[1], opensfm[0], color="red", lw=1.0, label="Visual-SLAM")
    traj.set_aspect('equal')
    annotations = [" t = 0 [s]", " t = 2 [s]", " t = 4 [s]", " t = 6 [s]", " t = 8 [s]", " t = 10 [s]", " t = 12 [s]", " t = 14 [s]"]
    traj.legend(fancybox=False, shadow=False, edgecolor='black')
    for i, label in enumerate(annotations):
        plt.annotate(label, (opensfm[1][0::29][i], opensfm[0][0::29][i]))
    traj.set_ylabel("Depth direction [m]")
    traj.set_xlabel("Lateral direction [m]")
    traj.set_title("Trajectory")
    #plt.xlim(-60, 50)
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/trajectory.png')
    plt.show()

def showX(groundtruth, opensfm, colmap, orbslam):
    fig, x = plt.subplots()
    time = Nx[0:221, 0]
    time2x = N2x
    x.plot(time, groundtruth[0], color="black", lw=0.5, label="Ground Truth")
    x.plot(time, opensfm[0], color="red", lw=0.5, label="OpenSfM")
    x.plot(time, colmap[0], color="blue", lw=0.5, label="Colmap")
    x.plot(time, orbslam[0][0:221], color="green", lw=0.5, label="ORB-SLAM2")
    x.legend(fancybox=False, shadow=False, edgecolor='black')
    x.set_xlabel("Time [s]")
    x.set_ylabel("Lateral direction [m]")
    x.set_title("Trajectory of depth direction")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/depth.png')
    plt.show()

def showY(groundtruth, opensfm, colmap, orbslam):
    fig, y = plt.subplots()
    time = Nx[0:221, 0]
    time2x = N2x
    y.plot(time, groundtruth[1], color="black", lw=0.5, label="Ground Truth")
    y.plot(time, opensfm[1], color="red", lw=0.5, label="OpenSfM")
    y.plot(time, colmap[1], color="blue", lw=0.5, label="Colmap")
    y.plot(time, orbslam[1][0:221], color="green", lw=0.5, label="ORB-SLAM2")
    y.legend(fancybox=False, shadow=False, edgecolor='black')
    y.set_xlabel("Time [s]")
    y.set_ylabel("Depth direction [m]")
    y.set_title("Trajectory of lateral direction")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/lateral.png')
    plt.show()

def getTranslationerror(groundtruth, x, y):
    translation_t = []
    translation = []
    translation_t1 = []
    translation1 = []
    error = []
    for i in range(frame - 1):
        translation_t1.append(np.sqrt((groundtruth[0][i + 1] - groundtruth[0][i]) ** 2 + (groundtruth[1][i + 1] - groundtruth[1][i]) ** 2))
        translation1.append(np.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2))
        error.append(100 * np.abs(translation_t1[i] - translation1[i]) / translation_t1[i])

    for i in range(frame):
        translation_t.append(np.sqrt((x[i] - groundtruth[0][i])**2 + (y[i] - groundtruth[1][i])**2))
        translation.append(np.sqrt(x[i] ** 2 + y[i] ** 2))



    return error, translation_t, translation

def showTranslationError(groundtruth, opensfm, colmap, orbslam):
    fig, te = plt.subplots()
    time = Nx[0:220, 0]
    te.plot(time, getTranslationerror(groundtruth, opensfm[0], opensfm[1])[0], color="red", lw=0.5, label="OpenSfM")
 #   te.plot(time, getTranslationerror(groundtruth, colmap[0], colmap[1])[0], color="blue", lw=0.5, label="Colmap")
    #te.plot(time, getTranslationerror(groundtruth, orbslam[0], orbslam[1])[0], color="green", lw=0.5, label="ORB-SLAM2")
    te.legend(fancybox=False, shadow=False, edgecolor='black')
    te.set_xlabel("Time [s]")
    te.set_ylabel("Translation Error [%]")
    te.set_title("Translation Error every timestamps")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/translationerror.png')
    plt.show()



def showRoll(groundtruth, opensfm):
    fig, roll = plt.subplots()
    time = Nx[:, 0]
    time2x = N2x
    roll.plot(time, groundtruth[2], color="black", lw=0.5, label="Ground Truth")
    roll.plot(time, opensfm[2], color="red", lw=0.5, label="OpenSfM")
    roll.legend(fancybox=False, shadow=False, edgecolor='black')
    roll.set_xlabel("Time [s]")
    roll.set_ylabel("Roll angle [deg]")
    roll.set_title("Roll angle")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/roll.png')
    plt.show()
    mae_opensfm = mean_absolute_error(groundtruth[2], opensfm[2])
    print("Roll mae_opensfm", mae_opensfm)

def showPitch(groundtruth, opensfm):
    fig, pitch = plt.subplots()
    time = Nx[:, 0]
    time2x = N2x
    pitch.plot(time, groundtruth[3], color="black", lw=0.5, label="Ground Truth")
    pitch.plot(time, opensfm[3], color="red", lw=0.5, label="OpenSfM")
    pitch.legend(fancybox=False, shadow=False, edgecolor='black')
    pitch.set_xlabel("Time [s]")
    pitch.set_ylabel("Pitch angle [deg]")
    pitch.set_title("Pitch angle")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/pitch.png')
    plt.show()
    mae_opensfm1 = mean_absolute_error(groundtruth[3], opensfm[3])
    print("Pitch mae_opensfm", mae_opensfm1)

def showYaw(groundtruth, opensfm):
    fig,yaw = plt.subplots()
    time = Nx[:, 0]
    time2x = N2x
    yaw.plot(time, groundtruth[4], color="black", lw=0.5, label="Ground Truth")
    yaw.plot(time, opensfm[4], color="red", lw=0.5, label="OpenSfM")

    yaw.legend(fancybox=False, shadow=False, edgecolor='black')
    yaw.set_xlabel("Time [s]")
    yaw.set_ylabel("Yaw angle [deg]")
    yaw.set_title("Yaw angle")
    plt.grid(True)
    plt.savefig('/home/sora-desktop/output/plotimage/yaw.png')
    plt.show()
    mae_opensfm2 = mean_absolute_error(groundtruth[4], opensfm[4])
    print("Yaw mae_opensfm", mae_opensfm2)

def showShots(groundtruth):
    fig, traj = plt.subplots()
    traj.scatter(groundtruth[1], groundtruth[0], s=1, color="black", label="Ground Truth")
    traj.scatter(groundtruth[1][0::29], groundtruth[0][0::29], s=10, color="red")
    traj.set_aspect('equal')
    annotations = [" t = 0 [s]", " t = 2 [s]", " t = 4 [s]", " t = 6 [s]", " t = 8 [s]", " t = 10 [s]", " t = 12 [s]", " t = 14 [s]"]
    traj.legend(fancybox=False, shadow=False, edgecolor='black')
    traj.set_ylabel("Depth direction [m]")
    traj.set_xlabel("Lateral direction [m]")
    traj.set_title("Trajectory of ground truth every 2 seconds")
    plt.grid(False)
    for i, label in enumerate(annotations):
        plt.annotate(label, (groundtruth[1][0::29][i], groundtruth[0][0::29][i]))
    #plt.xlim(-5, 100)
    plt.savefig('/home/sora-desktop/output/plotimage/shots.png')
    plt.show()

def calcRT(opensfm):
    t = np.array(opensfm[5])
    R = np.array(opensfm[6])
    RT = []
    hoge = np.array([0,0,-1.0])
    for i in range(len(t)-1):
        #RT.append(np.dot(R[i].T, (t[i+1]-t[i])/(1e-8+np.linalg.norm(t[i+1]-t[i]))))
        #v = np.dot(R[i+1], (np.dot(R[i+1].T, t[i+1])-np.dot(R[i].T, t[i])))
        v = t[i+1] - np.dot(R[i+1], np.dot(R[i].T, t[i])) + hoge*1e-3
        RT.append(np.linalg.norm(v/(np.linalg.norm(v))-hoge))
#        RT.append(v/(np.linalg.norm(v)))
        #RT.append(np.dot(R[i+1].T, t[i+1])-np.dot(R[i].T, t[i]))
        #RT.append((t[i+1]-t[i])/(1e-8+np.linalg.norm(t[i+1]-t[i])))

    time = Nx[:, 0]
    fig, rt = plt.subplots()
#    rt.plot(time[0:223], np.array(RT).T[0], color="green", lw=0.5, label="RT[x]")
#    rt.plot(time[0:223], np.array(RT).T[1], color="red", lw=0.5, label="RT[y]")
#    rt.plot(time[0:223], np.array(RT).T[2], color="blue", lw=0.5, label="RT[z]")
    rt.plot(time[0:223], np.array(RT).T, color="red", lw=1.0, label="Normalized vehicle attitude")
    rt.legend(fancybox=False, shadow=False, edgecolor='black')
    rt.set_xlabel("Time [s]")
    rt.set_ylabel("Normalized vehicle attitude")
    #rt.set_title("Normalized vehicle attitude")
    plt.grid(True)
    plt.ylim(0, 0.6)
    plt.savefig('/home/sora-desktop/output/plotimage/RT.png')
    #plt.savefig('/home/sora-desktop/output/plotimage/yaw.png')
    plt.show()
#    print(np.array(RT).T[0][:].shape)

if __name__ == '__main__':
    print("↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓RESULTS↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓")

    N0 = np.loadtxt('/home/sora-desktop/Documents/datasets/denso_20201125_turn_left3/sensor.csv', delimiter=',', skiprows=1, usecols=[2, 3, 4, 5, 6, 7, 8])
    M0 = np.loadtxt('/home/sora-desktop/Documents/datasets/denso_20201125_turn_left3/imu.csv', delimiter=',', skiprows=1)
    #L0 = np.loadtxt('/home/sora-desktop/dataset/KeyFrameTrajectory_turnleft4.txt', delimiter=' ')
    L0 = np.loadtxt('/home/sora-desktop/ORB_SLAM3/KeyFrameTrajectory_28.txt', delimiter=' ')
    json_file0 = open('/home/sora-desktop/Documents/datasets/denso_20201125_turn_left3/reconstruction.json', 'r')
    C0 = np.loadtxt('/home/sora-desktop/colmap0/images_turnleft4.txt', delimiter=' ', skiprows=4, usecols=[0, 1, 2, 3, 4, 5, 6, 7])

    groundtruth = calcGroundTruth(N0, M0)
    orbslam = calcOrbslam(groundtruth, L0)
    opensfm = calcOpensfm(groundtruth, json_file0)
    colmap = calcColmap(groundtruth, C0)

    calcRT(opensfm)
    showTrajectory(groundtruth, opensfm)
    #showShots(groundtruth)
    #showX(groundtruth, opensfm, colmap, orbslam)
    #showY(groundtruth, opensfm, colmap, orbslam)
    #showTranslationError(groundtruth, opensfm, colmap, orbslam)
    #showRoll(groundtruth, opensfm)
    #showPitch(groundtruth, opensfm)
    #showYaw(groundtruth, opensfm)

    #print("mean_distance_error(orbslam)", np.mean(np.array(getTranslationerror(groundtruth, orbslam[0], orbslam[1])[1])))
    #print("mean_distance_error(opensfm)", np.mean(np.array(getTranslationerror(groundtruth, opensfm[0], opensfm[1])[1])))
    #print("mean_distance_error(colmap)", np.mean(np.array(getTranslationerror(groundtruth, colmap[0], colmap[1])[1])))
