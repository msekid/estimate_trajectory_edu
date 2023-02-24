import numpy as np
import overpy
import folium
import statistics
import math
from init import Init
import gps2coord, coord2gps
from scipy import interpolate


class ExtractRoads():
    def __init__(self, extract_dist):
        self.EXTRACT_DIST = extract_dist
        self.gps_t = Init().gps_t

    def get_coords(self, gps_med):
        return [float(gps_med[0]), float(gps_med[1])]
        
    def create_bbox(self, coords, edge_len):
        edge_len = self.EXTRACT_DIST
        lat = coords[0]
        lon = coords[1]
        earth_radius = 6.3781e6
        
        delta_lat = (edge_len/earth_radius) * (180/np.pi)
        south = lat-delta_lat/2
        north = lat+delta_lat/2
        
        delta_lon = delta_lat / np.cos(lat*np.pi/180)
        west = lon - delta_lon/2
        east = lon + delta_lon/2
        
        bbox = [south, west, north, east]
        return bbox

    #適当な緯度経度を指定（coords:中心となる緯度経度, bbox:東西南北の緯度経度）
    #bboxさえ指定すればよい

    def makeRoads(self):

        api = overpy.Overpass()
        gps_t = self.gps_t
        len_n = int(Init().n_frame/2)
        gps_med_lon = np.array(gps_t).T[0][len_n]
        gps_med_lat = np.array(gps_t).T[1][len_n]
        coords = self.get_coords([gps_med_lon, gps_med_lat])
        #print(coords, "coords")
        bbox = self.create_bbox(coords, self.EXTRACT_DIST)
        #print(bbox)

        #道路の緯度経度をとってくる
        query2 = """
        [out:json][bbox:{0[0]},{0[1]},{0[2]},{0[3]}];
        (
        way["highway"="motorway"];
        way["highway"="trunk"];
        way["highway"="primary"];
        way["highway"="secondary"];
        way["highway"="tertiary"];
        way["highway"="unclassified"];
        way["highway"="residential"];
        way["highway"="service"];
        way["highway"="road"];
        );
        out center;
        """.format(bbox)
        result2 = api.query(query2)


        #m = folium.Map(location=coords, zoom_start=20)
        #m.save('output/map0.html')
        #print(result2.ways, "result2")
        return result2
    def latlon2coord(self, est):
        coord_t = []
        #print(len(est), "lenest")
        #print(est, "est")
        for i in range(len(np.array(est))):
            coord = gps2coord.calc_xy(np.array(est)[i][0], np.array(est)[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_t.append([coord[0], coord[1]])
        return coord_t
    
    def coord2gps(self, est):
        gps_est = []
        for i in range(len(np.array(est).T[0])):
            gps = coord2gps.calc_lat_lon(np.array(np.array(est).T[0])[i], np.array(np.array(est).T[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        #print(gps_est)
        return gps_est

    
    #print(latlon)
    def make_latlon_frommap(self):
        latlon_all = []
        result2 = self.makeRoads()
        m = folium.Map(location=self.gps_t[0], zoom_start=20)
        for way in result2.ways:
            nodes = way.get_nodes(resolve_missing=True)
            latlon = np.array([[float(node.lat), float(node.lon)] for node in nodes])
            coord = self.latlon2coord(latlon)
            dist = []
            total_dist = 0
            #for i in range()
            for xy in coord:
                dist.append(np.sqrt((xy[0] - coord[0][0])**2 + (xy[1] - coord[0][1])**2))
            print(dist ,"dist")
            latlon_x = interpolate.interp1d(dist, latlon.T[0], kind="linear")(np.arange(int(dist[-1])))
            latlon_y = interpolate.interp1d(dist, latlon.T[1], kind="linear")(np.arange(int(dist[-1])))
            latlon_ = np.vstack([latlon_x, latlon_y]).T
            print(latlon_)
            latlon_all.append(latlon_)
            #latlon_all.append(latlon)
            
            #print(latlon)
            
            #folium.PolyLine(latlon_, color='blue', weight=1.0).add_to(m)
            for data in latlon_:
                #print(data, "data")

                folium.Circle(data.tolist(), radius=1, color='blue', fill=False).add_to(m)
            #print(np.array(latlon_all), "lenlatlonall")
        m
        m.save('output/road_intered.html')
        return latlon_all
#ExtractRoads(extract_dist=300).make_latlon_frommap()