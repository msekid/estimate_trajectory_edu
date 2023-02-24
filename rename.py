import glob
import os
import numpy as np

# 拡張子.txtのファイルを取得する
path = '*.png'
i = 0
#L = np.loadtxt('/home/sora-desktop/dataset/drive-download-20220307T065449Z-001/denso_20201125_curve1_ine/mav0/cam0/data.csv', delimiter=' ', skiprows=1)
#print(L)
# txtファイルを取得する
flist = sorted(glob.glob(path))
print('変更前')
print(flist)



# ファイル名を一括で変更する
for file in flist:
    os.rename(file, str(i) + '.txt')
    i += 1

list = glob.glob(path)
print('変更後')
print(list)
