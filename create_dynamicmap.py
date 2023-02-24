import numpy as np
import folium
from folium.plugins import TimestampedGeoJson
from init import Init
from calc_traj import CalcTraj
from calc_gps import CalcGPS

class CreateDynamicMap():
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
    
    def createTime(self):
        times = []
        output = []
        for i in range(len(Init().time)):
            #times.append(i)
            times.append("T"+str(Init().time[i][3]).zfill(2)+":"+str(Init().time[i][2]).zfill(2)+":"+str(Init().time[i][1]).zfill(2))
            output.append("2022-06-15"+times[i])
            print(output)
        return output
    
    def saveFile(self):
        with open('timestamp.txt', 'w') as f:
            for i in CreateDynamicMap().createTime():
                f.write("%s\n" % i)
    
    
    def importFile(self):
        with open('timestamp.txt', 'r') as f:
            timestamp_list = f.read().split("\n")
        timestamp_list.pop(-1)
        #print(timestamp_list)
        return timestamp_list

    
    def drawMap(self):
        m = folium.Map(location=self.gps_t[0], zoom_start=20)
        lines = [
            {
                "coordinates":self.gps_t,
                "dates":CreateDynamicMap().importFile(),
                "color":"black"
            },
            {
                "coordinates":self.gps_slam,
                "dates":CreateDynamicMap().importFile()[8::7],
                "color":"green"
            },
            {
                "coordinates":self.gps_sfm,
                "dates":CreateDynamicMap().importFile()[8::7],
                "color":"red"
            },
        ]
        features = [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": line["coordinates"],
                },
                "properties": {
                    "times": line["dates"],
                    "style": {
                        "color": line["color"],
                        "weight": line["weight"] if "weight" in line else 5,
                    },
                },
            }
            for line in lines
        ]

        TimestampedGeoJson(
            {
                "type": "FeatureCollection",
                "features": features,
            },
            add_last_point=True,
        ).add_to(m)
        m.save('output/map_dynamic.html')
        m

if __name__ == '__main__':
    print(len(CreateDynamicMap().importFile()[8::7]))