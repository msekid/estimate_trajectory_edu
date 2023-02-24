from PIL import Image

import glob
import os
import numpy as np

# 拡張子.txtのファイルを取得する
path = '*.jpg'


# txtファイルを取得する
flist = glob.glob(path)


for file in flist:
    file_name = os.path.splitext(os.path.basename(file))[0]
    #im = Image.open(file)
    #im.save(file_name+'.png')
    print(file_name)
    os.remove(file)