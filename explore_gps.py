import numpy as np
from init import Init
from calc_traj import CalcTraj
from calc_gps import CalcGPS
import gps2coord, coord2gps
import folium
import matplotlib.pyplot as plt


class ExploreGPS():
    def __init__(self):

        self.gps_t = Init().gps_t
    
    def latlon2coord(self, est):
        coord_t = []
        for i in range(len(np.array(est))):
            coord = gps2coord.calc_xy(np.array(est)[i][0], np.array(est)[i][1], self.gps_t[0][0], self.gps_t[0][1])
            coord_t.append([coord[0], coord[1]])
        return coord_t
    
    def coord2gps(self, est):
        gps_est = []
        for i in range(len(np.array(est).T[0])):
            gps = coord2gps.calc_lat_lon(np.array(np.array(est).T[0])[i], np.array(np.array(est).T[1])[i], self.gps_t[0][0], self.gps_t[0][1])
            gps_est.append([gps[0], gps[1]])
        return gps_est

    def shift_gps(self, est, dist_x, dist_y):
        coord_shifted = []
        for i in range(len(np.array(est))):
            coord_shifted.append([est[i][0] + dist_x, est[i][1] + dist_y])
        return coord_shifted
    
    def scatter_gps(self, est):
        coord_scattered = []
        coord_scattered_ = []
        coord_scattered_all = []
        dist_x = np.arange(-100, 100, 10)
        dist_y = np.arange(-100, 100, 10)

        for dx in dist_x:
            for dy in dist_y:
                coord_scattered.append(self.shift_gps(est, dx, dy))
        return coord_scattered        
            

    def integratePoints_from_csv(self):
        coord_map = np.loadtxt('extracted_roads.csv', encoding="shift-jis", delimiter=',')
        return coord_map
    
    def calcDistance(self, coord_smrc):
        coord_map = self.integratePoints_from_csv()
        
        min = []
        dist_min = []
        for i in np.array(np.array(coord_smrc)):
            dist = []
            for j in coord_map:
                dist.append(np.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2))
            min.append(coord_map[np.argmin(dist)])
            dist_min.append([np.min(dist), i, coord_map[np.argmin(dist)]])
        return np.array(min).T

    def calcDistance_max(self, coord1, coord2):
        dist = []
        coord1 = np.array(coord1).T
        print(coord1.shape, "coord1")
        print(np.array(coord2).shape, "coord2")
        
        '''
        coord1_gpu = cp.asarray(coord1)
        coord2_gpu = cp.asarray(coord2)

        for i, j in zip(coord1_gpu, coord2_gpu):
            dist.append((cp.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2)).get())

        '''
        for i, j in zip(np.array(coord1), np.array(coord2)):
            dist.append(np.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2))
        return np.average(dist)

    def calcDistance_from_smrc(self, coord):
        dist = []
        coord_smrc = self.latlon2coord(self.gps_t)
        #print(coord_smrc)
        #for i, j in zip(np.array(coord_smrc), coord):
        #    dist.append(np.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2))
        i = np.array(coord_smrc)
        j = np.array(coord)
        center = int(Init().n_frame/2)
        dist = np.sqrt((i[center][0] - j[center][0])**2 + (i[center][1] - j[center][1])**2)
        return dist

    def explore_gps(self, threshold):
        gps_smrc = self.latlon2coord(self.gps_t)
        coord_scattered = self.scatter_gps(gps_smrc)
        dist = []
        print(coord_scattered, np.array(coord_scattered).shape)
        for i in range(len(np.array(coord_scattered))):
            map_selected = self.calcDistance(coord_scattered[i])
            dist.append(self.calcDistance_max(map_selected, coord_scattered[i]))
            print(i, coord_scattered.index)
        np.save('output/dist', np.array(dist))
        plt.rcParams['font.family'] = 'Times New Roman'
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        plt.rcParams['font.family'] = 'Times New Roman'
        ax.hist(dist, bins=30)
        ax.set_title('Distribution of trajectories searched for multiple points')
        ax.set_xlabel('Average distance to road network point cloud')
        ax.set_ylabel('The number of trajectories')
        fig.savefig('output/hist.png')
        plt.savefig('output/histgram.png')
        #fig.show()
        '''
        fig, hist = plt.subplots()
        labels = ['The number of trajectory']
        plt.hist(dist, label=labels)
        plt.grid(True)
        plt.legend(fancybox=False, shadow=False, edgecolor='black')
        plt.savefig('histgram.png')
        '''

        index_threshold = []
        for i in dist:
            if (i < min(dist)*threshold):
                index_threshold.append(dist.index(i))
            else:
                continue
        coord_selected_ = []
        for i in index_threshold:
            coord_selected_.append(coord_scattered[i])
        dist_ = []
        for i in range(len(coord_selected_)):
            dist_.append(self.calcDistance_from_smrc(coord_selected_[i]))
        min_index_ = dist_.index(min(dist_))
        min_index = dist.index(min(dist))
        #coord_selected = coord_scattered[min_index]
        coord_selected = coord_selected_[min_index_]
        #print(self.coord2gps(coord_selected))
        #print(self.gps_t)
        m = folium.Map(location=self.gps_t[0], zoom_start=20)
        for data in self.gps_t:
            folium.Circle(data.tolist(), radius=2, color='black', fill=False).add_to(m)
        #folium.PolyLine(self.gps_t, color='black', weight=3.0).add_to(m)
        
        for i in range(len(coord_selected_)):
            #if(i % 3 == 0):
                #folium.PolyLine(np.array(self.coord2gps(coord_scattered[i])).tolist(), color='blue', weight=1.0).add_to(m)
                folium.PolyLine(np.array(self.coord2gps(coord_selected_[i])).tolist(), color='blue', weight=1.0).add_to(m)
        folium.PolyLine(np.array(self.coord2gps(coord_selected)).tolist(), color='red', weight=4.0).add_to(m)
        m
        m.save('output/explore_threshold_1.5.html')
        return np.array(self.coord2gps(coord_selected)), coord_selected_, coord_selected

#print(ExploreGPS().explore_gps(1.2))
