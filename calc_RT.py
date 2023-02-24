import numpy as np
from init import Init
import matplotlib.pyplot as plt
from calc_traj import CalcTraj

class CalcRT():
    def __init__(self):
        self.N0 = Init().N0
        self.M0 = Init().M0
        self.L0 = Init().L0
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

        for i in range(len(t)-1):
            v = t[i+1] - np.dot(R[i+1], np.dot(R[i].T, t[i]))
            if (v[0] == 0 and v[1] == 0 and v[2] == 0):
                v_norm = 0
            else:
                v_norm = v/np.linalg.norm(v)
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("sfm", np.array(RT).T)
        return np.array(RT).T
    
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
        

        for i in range(len(t)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])
            v_norm = v/np.linalg.norm(v)
            RT.append(np.linalg.norm(v_norm-v_mean))
        #print("slam", np.array(RT).T)
        return np.array(RT).T


    def showRT(self):
        time = Init().Nx
        fig, rt = plt.subplots()
        RT_sfm = CalcRT().calcRT_sfm()
        RT_orbslam = CalcRT().calcRT_orbslam(self.L0)
        rt.plot(time[1:], np.array(RT_sfm).T, color="red", lw=1.0, label="Normalized vehicle attitude from OpenSfM")
        rt.plot(self.orbslam[6][1:], np.array(RT_orbslam).T, color="green", lw=1.0, label="Normalized vehicle attitude from ORB-SLAM2")
        rt.legend(fancybox=False, shadow=False, edgecolor='black')
        rt.set_xlabel("Time [s]")
        rt.set_ylabel("Normalized vehicle attitude")
        plt.grid(True)
        #plt.ylim(0, 0.6)
        plt.savefig('output/plotimage/RT.png')
        #plt.show()