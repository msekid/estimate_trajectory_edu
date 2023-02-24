import folium
import numpy as np
from folium.plugins import TimestampedGeoJson
import glob
from calc_traj import CalcTraj
from calc_gps import CalcGPS
import gps2coord, coord2gps
from init import Init
import osmnx as ox

class ExtractStreetNet():
    def __init__(self):

        self.groundtruth = CalcTraj().calcGroundTruth(Init().N0, Init().M0)
        self.orbslam = CalcTraj().calcOrbslam(self.groundtruth, Init().L0)
        self.opensfm = CalcTraj().calcOpensfm(self.groundtruth, Init().json_file0)

        self.gps_t = Init().gps_t
        self.gps_slam = np.array(CalcGPS().coord2gps_est(self.orbslam))
        self.gps_sfm = np.array(CalcGPS().coord2gps_est(self.opensfm))

        self.gps_t = self.gps_t.tolist()
        self.gps_slam = self.gps_slam.tolist()
        self.gps_sfm = self.gps_sfm.tolist()

    def extract(self):
        G = ox.graph_from_point((self.gps_t[0]))
        ox.plot_graph(G)

ExtractStreetNet().extract()

