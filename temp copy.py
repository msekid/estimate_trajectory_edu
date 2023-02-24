import numpy as np
import overpy
import folium
import statistics
import math
from init import Init


def get_coords(gps_med):
    '''
    result = api.query("""(
        node["name"~"{name}"];
        way["name"~"{name}"];
        );
        out center;""".format(name = search_name))
    '''
    '''
    if len(result.nodes)==0:
        print("No match found for", search_name)
        return None
    else:
        node = result.nodes[0]
    '''
    return [float(gps_med[0]), float(gps_med[1])]
    
def create_bbox(coords, edge_len = 100):
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
api = overpy.Overpass()
gps_t = Init().gps_t
len_n = int(Init().n_frame/2)
gps_med_lon = gps_t.T[0][len_n]
gps_med_lat = gps_t.T[1][len_n]
print(gps_med_lon, gps_med_lat)
coords = get_coords([gps_med_lon, gps_med_lat])
bbox = create_bbox(coords)

print("Coordinates:", coords)
print("Bounding box:", bbox)


#道路の緯度経度をとってくる
query = """
[out:json][bbox:{0[0]},{0[1]},{0[2]},{0[3]}];
(
way["highway"="motorway"];
);
out center;
""".format(bbox)

# 参考：自専道以外を抽出するためのクエリ
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

result = api.query(query)
result2 = api.query(query2)
'''
print("Type of result:", type(result))
print("Number of ways:", len(result.ways))
print("Example of a way:", result.ways[0])
'''
print("Type of result:", type(result2))
print("Number of ways:", len(result2.ways))
print("Example of a way:", result2.ways[0])

m = folium.Map(location=coords, zoom_start=20)
m.save('output/map0.html')
'''
for way in result2.ways:
    nodes = way.get_nodes(resolve_missing=True)
    latlon = np.array([[float(node.lat), float(node.lon)] for node in nodes])
    folium.PolyLine(latlon, color='red', weight=0.5).add_to(m)
'''
latlon_all = []
for way in result2.ways:
    nodes = way.get_nodes(resolve_missing=True)
    latlon = np.array([[float(node.lat), float(node.lon)] for node in nodes])
    latlon_all.append(latlon)
    #print(latlon)
    folium.PolyLine(latlon, color='blue', weight=1.0).add_to(m)
#print(latlon)
def make_latlon_frommap():
    return latlon_all

m.save('output/map_out.html')