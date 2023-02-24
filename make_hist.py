import matplotlib.pyplot as plt
import numpy as np




dist = np.load('output/dist.npy')
plt.rcParams['font.family'] = 'Times New Roman'
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.rcParams['font.family'] = 'Times New Roman'
ax.hist(dist, bins=30, color='green', rwidth=0.8)
#ax.set_title('Distribution of trajectories searched for multiple points')
ax.set_xlabel('Average distance to road network point cloud')
ax.set_ylabel('The number of trajectories')
fig.savefig('output/hist.png')
plt.savefig('output/histgram.png')