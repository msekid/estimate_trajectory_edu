from view_coord import ViewCoord
from calc_traj import CalcTraj
from create_staticmap import CreateStaticMap
from create_dynamicmap import CreateDynamicMap
from init import Init
from calc_RT__ import CalcRT
from optimize_traj7 import OptimizeTraj
import numpy as np
from equalize_est import Equalize


#from create_staticmap_map import CreateStaticMap_
#from save_data import SaveData

class Main():
    def __init__(self):
        a = Init()
        self.N0 = a.N0
        self.M0 = a.M0
        self.gps_t = a.gps_t
        self.L0 = a.L0
        self.json_file0 = a.json_file0
        self.droid = a.droid
        self.groundtruth = CalcTraj().calcGroundTruth(self.N0, self.M0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, self.json_file0)
        self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, self.droid)
        self.optimized = OptimizeTraj().calcOptimizeTraj()
        if (len(self.L0) == 0):
            self.orbslam = self.droidslam
        else:
            self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        self.equalizedDROID = Equalize().averagedSLAM(a.droid0, a.droid1, a.droid2, a.droid3, a.droid4, a.droid5, a.droid6, a.droid7, a.droid8, a.droid9, 'DROIDSLAM')

        if (len(a.L) == 0):
            self.equalizedORB = self.equalizedDROID
        else:
            self.equalizedORB = Equalize().averagedSLAM(a.L0, a.L1, a.L2, a.L3, a.L4, a.L5, a.L6, a.L7, a.L8, a.L9, 'ORBSLAM')

        
if __name__ == '__main__':
    print("output")
    
    #CreateDynamicMap().drawMap()
    a = Main()
    #print(a.optimized[0], "shape")
    b = ViewCoord()
    #a.showTrajectory(a.groundtruth, a.opensfm, a.orbslam, a.doidslam)
    #CreateStaticMap_().drawPolyLine()
    #ViewCoord().showTrajectory(Main().groundtruth, Main().opensfm, Main().orbslam)
    b.showTrajectory(a.groundtruth, a.opensfm, a.orbslam, a.droidslam, a.optimized, a.equalizedORB, a.equalizedDROID)
    b.showRoll(a.groundtruth,a.opensfm, a.orbslam, a.droidslam, a.optimized, a.equalizedORB, a.equalizedDROID)
    b.showPitch(a.groundtruth,a.opensfm, a.orbslam, a.droidslam, a.optimized, a.equalizedORB, a.equalizedDROID)
    b.showYaw(a.groundtruth,a.opensfm, a.orbslam, a.droidslam, a.optimized, a.equalizedORB, a.equalizedDROID)
    b.showZ(a.groundtruth,a.opensfm, a.orbslam, a.droidslam, a.optimized)
    OptimizeTraj().showRT()

