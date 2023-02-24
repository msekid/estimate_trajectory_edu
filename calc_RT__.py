import numpy as np
from init import Init
import matplotlib.pyplot as plt
from calc_traj import CalcTraj
from scipy import integrate, signal, interpolate
import math

class CalcRT():
    def __init__(self):
        self.time = Init().Nx
        self.L0 = Init().L0
        self.L1 = Init().L0
        if (len(self.L0) == 0):
            self.t_orb = Init().Nx
        else:
            self.t_orb = Init().L0[:, 0]
        self.N0 = Init().N0
        self.M0 = Init().M0
        
        self.droid = Init().droid
        self.json_file0 = Init().json_file0
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, self.droid)
        if (len(self.L0) == 0):
            self.orbslam = self.droidslam
        else:
            self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)

    
    
    def euler2RotationMatrix(self, roll, pitch, yaw):
        R_ = []
        R_x = []
        R_y = []
        R_z = []
        for i in range(Init().n_frame):
            '''
            p_ = (np.pi+roll[i])
            r_ = pitch[i]
            ya_ = -(np.pi+yaw[i])
            '''

            p_ = roll[i]
            r_ = pitch[i]
            ya_ = -yaw[i] #- np.pi
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

    
    def calcRT(self, x, y, z, roll, pitch, yaw):
        R = np.array(self.euler2RotationMatrix(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)))
        r1 = np.zeros(len(self.time))
        p1 = np.zeros(len(self.time))
        ya1 = np.zeros(len(self.time))
        for i in range(len(self.time)):
            eul = np.rad2deg(CalcTraj.rotationMatrixToEulerAngles(self, R[i]))
            r1[i] = -eul[0]
            p1[i] = -eul[1]
            ya1[i] = -eul[2]
            #print(roll[i], r1[i], "roll")
            #print(pitch[i], p1[i], "pitch")
            #print(yaw[i], ya1[i], "yaw")
        t = np.vstack([x, y, z]).T
        RT = []
        v_sum = []
        for i in range(len(t)-1):
            v = np.dot(np.array(R[i+1]).T, t[i+1] - t[i])

            v_sum.append(v)
        v_mean = np.average(v_sum, axis=0)

        v_mean = v_mean/np.linalg.norm(v_mean)

        v_sfm = []
        for i in range(len(t)):
            if (i == 0):
                v_sfm.append([0, 0, 0])
                RT.append(0)
            else:
                v = np.dot(np.array(R[i]).T, t[i] - t[i-1])
                if (v[0] == 0 and v[1] == 0 and v[2] == 0):
                    v_norm = 0
                else:
                    v_norm = v/np.linalg.norm(v)
                v_sfm.append(v)
                RT.append(np.linalg.norm(v_norm-v_mean))
        #print(np.array(RT).shape, t.shape, np.array(v_sfm).shape)
        #print(np.vstack([np.average(roll), np.average(pitch), np.average(yaw)]).T, "angle")
        #print(np.vstack(np.average(t,axis = 0)))

        return t, v_sfm, R, RT

'''
    def showRT2(self):
        time = Init().Nx
        fig, rt = plt.subplots(figsize=(32, 8))
        RT_sfm = self.calcRT(self.opensfm[0], self.opensfm[1], self.opensfm[9], self.opensfm[4], self.opensfm[3], self.opensfm[2])[3]
        RT_droidslam = self.calcRT(self.droidslam[0], self.droidslam[1], self.droidslam[2], self.droidslam[3], self.droidslam[4], self.droidslam[5])[3]
        if (len(Init().L0) == 0):
            RT_orbslam = RT_droidslam
        else:
            RT_orbslam = self.calcRT(self.orbslam[0], self.orbslam[1], self.orbslam[2], self.orbslam[3], self.orbslam[4], self.orbslam[5])[3]
        #RT_droidslam = self.calc_RT_only_droid()[0]
        
        rt.plot(time[:], np.array(RT_sfm).T, color="red", lw=2, label="Normalized vehicle attitude from OpenSfM")
        rt.plot(time[:], np.array(RT_orbslam).T, color="green", lw=2, label="Normalized vehicle attitude from ORB-SLAM2")
        rt.plot(time[:], np.array(RT_droidslam).T, color="blue", lw=2, label="Normalized vehicle attitude from DROID-SLAM")
        rt.legend(fancybox=False, shadow=False, edgecolor='black')
        rt.set_xlabel("Time [s]")
        rt.set_ylabel("Normalized vehicle attitude")
        plt.grid(True)
        #plt.ylim(0, 0.6)
        plt.savefig('output/opted/RT.png')
        #plt.show()
'''