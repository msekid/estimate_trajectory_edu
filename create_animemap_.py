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
#import cartopy.io.img_tiles as cimgt
import matplotlib.pyplot as plt
from staticmap import StaticMap
import matplotlib.pyplot as plt
from math import log, tan, pi, cos
from PIL import ImageFont, ImageDraw


class CreateAnimeMap():
    def __init__(self):
        self.gps_rotated = np.loadtxt('optimizedGPS.csv', delimiter=",")
        self.gps_optimized = self.gps_rotated

    def lon_to_pixel(self, lon, map):
        # 経度→タイル番号
        if not (-180 <= lon <= 180):
            lon = (lon + 180) % 360 - 180
        x = ((lon + 180.) / 360) * pow(2, map.zoom)
        # タイル番号→キャンバスの座標
        pixel = (x - map.x_center) * map.tile_size + map.width / 2
        return int(round(pixel))

    def lat_to_pixel(self, lat, map):
        # 緯度→タイル番号
        if not (-90 <= lat <= 90):
            lat = (lat + 90) % 180 - 90
        y = (1 - log(tan(lat * pi / 180) + 1 / cos(lat * pi / 180)) / pi) / 2 * pow(2, map.zoom)
        # タイル番号→キャンバスの座標
        pixel = (y - map.y_center) * map.tile_size + map.height / 2
        return int(round(pixel))

    def draw_txt_example(self):
        # 地理院地図タイル
        map = StaticMap(1280, 720, url_template="https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png")
        # 帯広を中心にする
        num =int(Init().n_frame/2)
        #print(self.gps_optimized[num][1])
        img = map.render(zoom=18, center=[self.gps_optimized[num][1], self.gps_optimized[num][0]])

        # 日本語フォント
        #font = ImageFont.truetype("C://Windows/Fonts/yugothb.ttc", 32)
        draw = ImageDraw.Draw(img)
        for point in self.gps_optimized:
            print(point)
            x = self.lon_to_pixel(point[1], map)
            y = self.lat_to_pixel(point[0], map)
            # テキストを点に対して中央揃えするためにテキストのピクセルサイズを取得
            #str_w, str_h = draw.textsize(point[0], font=font)
            # 駅名の描画
            #if x >= 0 and x < img.width and y >= 0 and y < img.height:
            #    draw.text((x-str_w//2, y-str_h//2), point[0], (64, 64, 255), font=font)
            draw.point([(y, x)], fill=(255, 0, 0))
        plt.imshow(img)
        plt.show()
            


a = CreateAnimeMap()

a.draw_txt_example()