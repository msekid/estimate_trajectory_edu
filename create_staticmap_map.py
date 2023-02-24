import folium
import numpy as np
from folium.plugins import TimestampedGeoJson
import glob
from calc_traj import CalcTraj
from calc_gps import CalcGPS
import gps2coord, coord2gps
from init import Init
from PIL import Image, ImageDraw
from optimize_traj7 import OptimizeTraj
from extract_roadnet import Extractmap
import aspose.words as aw
from pdf2image import convert_from_path

class CreateStaticMap_():
    def __init__(self):

        self.groundtruth = CalcTraj().calcGroundTruth(Init().N0, Init().M0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, Init().L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, Init().json_file0)
        #self.droidslam = CalcTraj().calcDroidslam(self.groundtruth, Init().droid)
        self.optimized = OptimizeTraj().calcOptimizeTraj()
        self.road = Extractmap().calcDistance()[1]

        self.gps_t = Init().gps_t
        self.gps_slam = np.array(Extractmap().coord2gps_est(self.orbslam))
        self.gps_sfm = np.array(Extractmap().coord2gps_est(self.opensfm))
        #self.gps_droid = np.array(Extractmap().coord2gps_est(self.droidslam))
        self.gps_optimized = np.array(Extractmap().coord2gps_est(self.optimized))
        self.gps_road = np.array(Extractmap().coord2gps_est(self.road))

        self.gps_t = self.gps_t.tolist()
        self.gps_slam = self.gps_slam.tolist()
        self.gps_sfm = self.gps_sfm.tolist()
        #self.gps_droid = self.gps_droid.tolist()
        self.gps_optimized = self.gps_optimized.tolist()
        self.gps_road = self.gps_road.tolist()


    def createTime(self):
        times = []
        output = []
        for i in range(len(Init().time)):
            times.append("T"+str(Init().time[i][3])+":"+str(Init().time[i][2])+":"+str(Init().time[i][1]))
            output.append(Init().day+times[i])
        return output



    def drawPolyLine(self):
        m = folium.Map(location=self.gps_t[0], zoom_start=20)
        #folium.PolyLine(self.gps_t, color='black', weight=0.5).add_to(m)
        folium.PolyLine(self.gps_slam, color='green', weight=0.5).add_to(m)
        folium.PolyLine(self.gps_sfm, color='red', weight=0.5).add_to(m)
        #folium.PolyLine(self.gps_droid, color='blue', weight=3).add_to(m)
        #folium.PolyLine(self.gps_road, color='blue', weight=0.5).add_to(m)
        folium.PolyLine(self.gps_optimized, color='magenta', weight=1).add_to(m)
        m
        m.save('output/map_.html')
    
    def html2png(self):
        doc = aw.Document('output/map_.html')
        doc.save('output/map_.pdf')
        pages = convert_from_path('output/map_.pdf', 500)
        for page in pages:
            page.save('output/map_.png', 'PNG')