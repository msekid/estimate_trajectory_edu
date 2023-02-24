import cv2
from init import Init
import sys



fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
im = cv2.imread('./png/0000.png')
h, w = im.shape[:2]
# output file name, encoder, fps, size(fit to image size)
video_original = cv2.VideoWriter('./original.mp4',fourcc, 14.0, (w, h))



for i in range(Init().n_frame):
    img_original = cv2.imread(('./png/{0:04d}.png').format(i))

    # add
    video_original.write(img_original)
    print(i)

video_original.release()