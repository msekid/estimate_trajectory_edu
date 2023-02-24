import requests
import numpy as np

data = np.loadtxt('final_output.csv', encoding="shift-jis", delimiter=',', usecols=[6, 7])



def getAltitude(latitude: float, longitude: float) -> float:
    url = 'http://api.openstreetmap.org/api/0.6/map?bbox={},{},{},{}'.format(longitude-0.0001,latitude-0.0001,longitude+0.0001,latitude+0.0001)
    response = requests.get(url)
    xml = response.text
    xml = xml.replace(' ', '')
    xml = xml.replace('\n', '')
    way = xml[xml.find('<way'):]
    print(way)
    tags = way[way.find('<tag'):way.find('</way')]
    if 'highway' not in tags:
        return None
    if 'ele' not in tags:
        return None
    elevation = float(tags[tags.find('ele="')+5:tags.find('"', tags.find('ele="')+5)])
    return elevation




def getAltitude_(latitude, longitude):
    # OpenStreetMapのAPIのURL
    url = f"https://api.openstreetmap.org/api/0.6/way?bbox={longitude-0.001},{latitude-0.001},{longitude+0.001},{latitude+0.001}"
    
    # APIにリクエストを送信し、レスポンスを取得する
    response = requests.get(url)
    
    # XMLを解析して高度情報を取得する
    altitude = None
    if response.status_code == 200:
        xml_data = response.content
        start_index = xml_data.find(b'<nd lat="')
        end_index = xml_data.find(b'"/>', start_index)
        node_data = xml_data[start_index:end_index+3].decode('utf-8')
        altitude_index = node_data.find('alt="')
        if altitude_index != -1:
            altitude_end_index = node_data.find('"', altitude_index+5)
            altitude = float(node_data[altitude_index+5:altitude_end_index])
    
    return altitude



def getAltitude__(latitude: float, longitude: float) -> float:
    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = f"""
    [out:json];
    node({latitude},{longitude});
    out;
    """
    response = requests.get(overpass_url, params={'data': overpass_query})
    data = response.json()
    try:
        altitude = float(data['elements'][0]['tags']['ele'])
    except KeyError:
        altitude = 0.0
    return altitude


def getlist(data):
    alt = []
    for i in range(len(data)):
        alt.append(getAltitude(data[i][0], data[i][1]))
        print(alt)
    #print(alt)
    return alt
getlist(data)

