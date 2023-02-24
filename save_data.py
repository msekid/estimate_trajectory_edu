import numpy as np
from calc_traj import CalcTraj
from init import Init
from optimize_traj7 import OptimizeTraj
from optimize_gps import OptimizeGPS
import folium

class SaveData():
    def __init__(self):
        self.groundtruth = CalcTraj().calcGroundTruth(Init().N0, Init().M0)
        self.L0 = Init().L0
        self.droid = Init().droid
        self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, self.droid)
        if (len(self.L0) == 0):
            self.orbslam = self.droidslam
        else:
            self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, self.L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, Init().json_file0)
        self.optimized = OptimizeTraj().calcOptimizeTraj()
        
        self.optimized_gps_2 = np.array(OptimizeGPS(extract_dist=300).optimizeGPS(self.optimized, 2)[0])
        self.optimized_gps = np.array(OptimizeGPS(extract_dist=300).optimizeGPS(self.optimized,10)[0])
        
        #self.optimized_gps_orb = np.array(OptimizeGPS(extract_dist=200).optimizeGPS(self.orbslam, 2)[0])
        #self.optimized_gps_sfm = np.array(OptimizeGPS(extract_dist=200).optimizeGPS(self.opensfm, 2)[0])
        self.gps_t = Init().gps_t
        #self.road_gps = np.array(OptimizeGPS(extract_dist=200).optimizeGPS(self.optimized, 2)[1])


    def saveData_toRyuchi(self):
        #data = np.vstack([self.opensfm[1], self.opensfm[0], self.opensfm[2], self.opensfm[3], self.opensfm[4]]).T
        #print(len(self.optimized[1]))
        #print(len(self.optimized[3][1:]))
        data = np.vstack([self.optimized[0], self.optimized[1], self.optimized[3][:], self.optimized[4][:], self.optimized[5][:]]).T
        np.savetxt("data.csv", data, delimiter=",")

    def saveData_gps(self):
        data = np.vstack([self.optimized_gps.T[0], self.optimized_gps.T[1]]).T
        data_10 = np.vstack([self.optimized_gps_2.T[0], self.optimized_gps_2.T[1]]).T
        np.savetxt("optimizedGPS.csv", data, delimiter=",")
        m = folium.Map(location=self.gps_t[0], zoom_start=20)
        for data1, data2 ,data3, in zip(np.array(self.optimized_gps), self.gps_t, np.array(self.optimized_gps_2)):
            folium.Circle(data2.tolist(), radius=2, color='black', fill=False).add_to(m)
            folium.Circle(data1.tolist(), radius=1, color='magenta', fill=False).add_to(m)
            folium.Circle(data3.tolist(), radius=1, color='orange', fill=False).add_to(m)
        m
        m.save('output/map_smrc.html')

    def saveData_toAka(self):
        data = np.vstack([self.road_gps.T[0], self.road_gps.T[1]]).T
        np.savetxt("road_gps.csv", data, delimiter=",")


    def saveData_FinalOutput(self):
        data1 = np.vstack([self.optimized[3], self.optimized[4], self.optimized[5], self.optimized[0], self.optimized[1], self.optimized[2], self.optimized_gps.T[0], self.optimized_gps.T[1]]).T
        np.savetxt("final_output.csv", data1, delimiter=",")
    
    def saveData_FinalOutput_orb(self):
        data2 = np.vstack([self.orbslam[3], self.orbslam[4], self.orbslam[5], self.orbslam[1], self.orbslam[0], self.orbslam[2], self.optimized_gps_orb.T[0], self.optimized_gps_orb.T[1]]).T
        np.savetxt("final_output_orb.csv", data2, delimiter=",")
    
    def saveData_FinalOutput_sfm(self):
        data3 = np.vstack([self.opensfm[4], self.opensfm[3], self.opensfm[2], self.opensfm[0], self.opensfm[1], self.opensfm[9], self.optimized_gps_sfm.T[0], self.optimized_gps_sfm.T[1]]).T
        np.savetxt("final_output_sfm.csv", data3, delimiter=",")

#SaveData().saveData_toRyuchi()
SaveData().saveData_gps()
#SaveData().saveData_FinalOutput()
#SaveData().saveData_toAka()
#SaveData().saveData_FinalOutput_orb()
#SaveData().saveData_FinalOutput_sfm()
