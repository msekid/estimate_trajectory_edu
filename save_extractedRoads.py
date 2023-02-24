import numpy as np
from temp import ExtractRoads
import gps2coord, coord2gps
from init import Init


class SaveExtractedRoads():
    def __init__(self, extract_dist):
        self.EXTRACT_DIST = extract_dist
        self.gps_t = Init().gps_t

    def integratePoints(self):
        integrated = []
        coord_map = []
        total = 0
        gps_map = np.array(ExtractRoads(self.EXTRACT_DIST).make_latlon_frommap())
        for i in range(len(gps_map)):
        #    total = total + gps_map[i].size
            for j in gps_map[i]:
                integrated.append(j)
                coord = gps2coord.calc_xy(j[0], j[1], self.gps_t[0][0], self.gps_t[0][1])
                coord_map.append([coord[0], coord[1]])
        coord_map = np.array(coord_map).T
        data = np.vstack([coord_map[0], coord_map[1]]).T
        np.savetxt("extracted_roads.csv", data, delimiter=",")
        return np.array(coord_map)

SaveExtractedRoads(extract_dist=300).integratePoints()