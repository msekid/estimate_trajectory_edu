import numpy as np
import glob
from scipy import interpolate

class Init():
    def __init__(self):
        self.N0 = np.loadtxt(glob.glob('estimated_*')[0], encoding="shift-jis", delimiter=',', skiprows=1, usecols=[1])
        self.M0 = np.loadtxt(glob.glob('ID*')[0], encoding="shift_jisx0213", delimiter=',', skiprows=1, usecols=[6, 7, 8])
        self.gps_t_ = np.loadtxt(glob.glob('ID*')[0], encoding="shift_jisx0213", delimiter=',', skiprows=1, usecols=[4, 5])
        self.n_frame = len(glob.glob("images/*.jpg"))
        index = np.where(self.gps_t_.T[0] != 0)[0]
        
        gps_t_x = interpolate.interp1d(index, self.gps_t_[index].T[0], fill_value=(self.gps_t_.T[0][index[0]], self.gps_t_.T[0][index[-1]]), bounds_error=False)(np.arange(self.n_frame))
        gps_t_y = interpolate.interp1d(index, self.gps_t_[index].T[1], fill_value=(self.gps_t_.T[1][index[0]], self.gps_t_.T[1][index[-1]]), bounds_error=False)(np.arange(self.n_frame))
        self.gps_t = np.vstack([gps_t_x, gps_t_y]).T

        self.L0 = np.loadtxt('KeyFrameTrajectory.txt', delimiter=' ')
        
        
        self.json_file0 = open('reconstruction.json', 'r')
        
        self.len_groundtruth = len(np.loadtxt(glob.glob('ID*')[0], encoding="shift_jisx0213", delimiter=',', skiprows=1, usecols=[4])) #ここを変更
        self.time_groundtruth = np.arange(self.len_groundtruth)*(1/14) #ここを変更
        self.Nx  = np.arange(self.n_frame)*(1/14)
        #self.day = np.loadtxt(glob.glob('ID*')[0], encoding="shift-jis", dtype="unicode", delimiter=',', skiprows=2, usecols=[27])[0].replace('/', '-')
        #self.time = np.loadtxt(glob.glob('ID*')[0], encoding="shift-jis", dtype="unicode", delimiter=',', skiprows=2, usecols=[16, 17, 18, 19])
        self.droid = np.vstack([np.load('tstamps.npy').T, np.load('poses.npy').T]).T

        self.L0 = np.loadtxt('OUTPUT_ORBSLAM/0.txt', delimiter=' ')
        self.L1 = np.loadtxt('OUTPUT_ORBSLAM/1.txt', delimiter=' ')
        self.L2 = np.loadtxt('OUTPUT_ORBSLAM/2.txt', delimiter=' ')
        self.L3 = np.loadtxt('OUTPUT_ORBSLAM/3.txt', delimiter=' ')
        self.L4 = np.loadtxt('OUTPUT_ORBSLAM/4.txt', delimiter=' ')
        self.L5 = np.loadtxt('OUTPUT_ORBSLAM/5.txt', delimiter=' ')
        self.L6 = np.loadtxt('OUTPUT_ORBSLAM/6.txt', delimiter=' ')
        self.L7 = np.loadtxt('OUTPUT_ORBSLAM/7.txt', delimiter=' ')
        self.L8 = np.loadtxt('OUTPUT_ORBSLAM/8.txt', delimiter=' ')
        self.L9 = np.loadtxt('OUTPUT_ORBSLAM/9.txt', delimiter=' ')

        self.droid0 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_0.npy').T, np.load('OUTPUT_DROIDSLAM/poses_0.npy').T]).T
        self.droid1 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_1.npy').T, np.load('OUTPUT_DROIDSLAM/poses_1.npy').T]).T
        self.droid2 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_2.npy').T, np.load('OUTPUT_DROIDSLAM/poses_2.npy').T]).T
        self.droid3 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_3.npy').T, np.load('OUTPUT_DROIDSLAM/poses_3.npy').T]).T
        self.droid4 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_4.npy').T, np.load('OUTPUT_DROIDSLAM/poses_4.npy').T]).T
        self.droid5 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_5.npy').T, np.load('OUTPUT_DROIDSLAM/poses_5.npy').T]).T
        self.droid6 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_6.npy').T, np.load('OUTPUT_DROIDSLAM/poses_6.npy').T]).T
        self.droid7 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_7.npy').T, np.load('OUTPUT_DROIDSLAM/poses_7.npy').T]).T
        self.droid8 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_8.npy').T, np.load('OUTPUT_DROIDSLAM/poses_8.npy').T]).T
        self.droid9 = np.vstack([np.load('OUTPUT_DROIDSLAM/tstamps_9.npy').T, np.load('OUTPUT_DROIDSLAM/poses_9.npy').T]).T
        self.L = []
        for data in [self.L0, self.L1, self.L2, self.L3, self.L4, self.L5, self.L6, self.L7, self.L8, self.L9]:
            if(len(data) == 0):
                continue
            else:
                self.L.append(data.tolist())
Init()
        