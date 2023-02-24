import numpy as np

#disps = np.load('disps.npy')
#images = np.load('images.npy')
poses = np.load('poses.npy')
tstamps = np.load('tstamps.npy')
#intrinsics =    np.load('intrinsics.npy')


#print(disps.shape)
#print(images.shape)
#print(tstamps)
#print(intrinsics.shape)
data = np.vstack([tstamps.T, poses.T]).T
def exportFile():
    with open('droid_slam.txt', 'w') as f:
        for i in poses:
            f.write("%s\n" % i)
#exportFile()
print(data[1:3])
