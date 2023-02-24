import numpy as np
import folium
from explore_gps import ExploreGPS

class SaveGPS():
    def __init__(self):
        self.gps = np.array(ExploreGPS().explore_gps(1.5))[0]

    def save(self):
        data = np.vstack([self.gps.T[0], self.gps.T[1]]).T
        np.savetxt("exploreGPS.csv", data, delimiter=",")

SaveGPS().save()
