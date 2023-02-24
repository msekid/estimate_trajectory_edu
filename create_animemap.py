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
from time import sleep as slp
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from webdriver_manager.firefox import GeckoDriverManager
from webdriver_manager.chrome import ChromeDriverManager
import os
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from PIL import Image


class CreateAnimeMap():
    def __init__(self):
        self.gps_rotated = np.loadtxt('optimizedGPS.csv', delimiter=",")
        self.gps_optimized = self.gps_rotated

    def plotGPS(self):
        map = folium.Map(location=self.gps_optimized[int(Init().n_frame/2)], zoom_start=20)
        #map = folium.Map(location=self.gps_optimized[0], tiles='https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png',attr='国土地理院', zoom_start=20)
        #map = folium.Map(location=self.gps_optimized[0], tiles='https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg',attr='国土地理院', zoom_start=20)
        num = 0
        for i in self.gps_optimized:
            print(list(i))
            folium.Circle(location=list(i), radius=0.1, color='lightgray', fill=False).add_to(map)

        for i in Init().gps_t:
            folium.Circle(location=list(i), radius=0.1, color='black', fill=False).add_to(map)
        
        for i in self.gps_optimized:
            folium.Circle(location=list(i), radius=1, color='red', fill=True).add_to(map)
            map.save('gps_map_.html')
            path = os.getcwd()
            print(path)
            URL = "file:///" + path + "/gps_map_.html"
            browser = webdriver.Chrome(executable_path=ChromeDriverManager().install())
            #driver.set_window_size(1280, 720)
            browser.get(URL)
            #win = browser.find_element(By.TAG_NAME,"html")
            #win.send_keys(Keys.CONTROL + Keys.SHIFT + "=")
            #win.send_keys(Keys.CONTROL + Keys.SHIFT + "=")
            #win.send_keys(Keys.CONTROL + Keys.SHIFT + "=")
            #win.send_keys(Keys.CONTROL + Keys.SHIFT + "=")
            slp(3)
            browser.save_screenshot(f'./png/{num:04d}.png')
            #slp(3)
            #img = Image.open(f'./png/{num:04d}.png')
            #img = img.convert('RGB') # RGBA(png)→RGB(jpg)へ変換
            #img.save(f'./jpg/{num:04d}.jpg', "JPEG", quality=95)
            num = num + 1
            browser.quit()

        

a = CreateAnimeMap()
a.plotGPS()