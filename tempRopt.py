import numpy as np
import csv
import pprint
import math
id = 565153


def rotationMatrixToEulerAngles(R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            pitch = math.atan2(R[2, 1], R[2, 2])
            roll = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            pitch = math.atan2(-R[1, 2], R[1, 1])
            roll = math.atan2(-R[2, 0], sy)
            yaw = 0
        return np.array([roll, pitch, yaw])

with open('/home/sora-lab/Desktop/output_tabata_1216/csv/' + str(id) + '.csv') as f:
    reader = csv.reader(f)
    l = [row for row in reader]

R_opt = np.load(str(id) + '.npy')

for i in range(len(R_opt)):
    eul = rotationMatrixToEulerAngles(R_opt[i])
    r1 = -eul[0]
    p1= eul[1] % 360 - 180
    ya1 = -eul[2]
    print('npy: {:.2}, {:.2}, {:.2}, csv: {:.2}, {:.2}, {:.2}'.format(r1, p1, ya1, float(l[i][0]),float(l[i][1]),float(l[i][2]) ))

