import numpy as np
from PIL import Image
import cv2
import os

ORIGINAL_FILE_DIR = "images/"
MASK_FILE_DIR = "pred_mask/"
OUTPUT_FILE_DIR = "masks/"

def createMask():
    mask = np.array(Image.open(path))
    buil = np.ones_like(mask)
    buil[mask == 1] = 0
    buil[mask == 2] = 0
    buil[mask == 3] = 0
    buil[mask == 4] = 0
    buil[mask == 5] = 0
    buil[mask == 8] = 0
    buil[mask == 15] = 0
    buil[mask == 30] = 0
    return buil*255

if __name__ == '__main__':
    files = os.listdir(MASK_FILE_DIR)
    for val in files:
        file_name = os.path.splitext(os.path.basename(val))[0]
        path = MASK_FILE_DIR + val
        cv2.imwrite(OUTPUT_FILE_DIR + file_name + '.jpg' + '.png', createMask())
