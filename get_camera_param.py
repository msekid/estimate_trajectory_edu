import numpy as np
import pandas as pd
import sys
import os
import shutil


args = sys.argv
shutil.copy('../config.yaml', 'config.yaml')
#camera_param = pd.ExcelFile('camera_param.xlsx')
#sheet_df = camera_param.parse('videoID_carame_parameters')
camera_id = args[1]
sheet_df = pd.read_excel('../camera_param.xlsx', sheet_name = 'videoID_carame_parameters')
print(args[1])
camera_type = sheet_df.at[sheet_df['VideoID'].tolist().index(int(camera_id)), 'CameraType']
#print(sheet_df.index.values)
#print(sheet_df.columns.values)
if (camera_type == '堀場試作品'):
    shutil.copy('../camera_models_overrides_horibatest.json', 'camera_models_overrides.json')
elif (camera_type == 'DRU-5010'):
    shutil.copy('../camera_models_overrides_DR-5010.json', 'camera_models_overrides.json')
elif (camera_type == 'DR-3031'):
    shutil.copy('../camera_models_overrides_DR-3031.json', 'camera_models_overrides.json')
elif (camera_type == 'DR-6200'):
    shutil.copy('../camera_models_overrides_DR-6200.json', 'camera_models_overrides.json')
elif (camera_type == 'DR-9100'):
    shutil.copy('../camera_models_overrides_DR-9100.json', 'camera_models_overrides.json')
else:
    print('NO CAMERA TYPE, PLEASE CHECK YOUR ID, OR REGISTER NEW ID INTO THE EXCEL FILE')