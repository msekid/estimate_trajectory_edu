# tracks.csvのデータから特徴点を読み込み、画像に合わせて表示させるプログラム

import os
import cv2
import csv
import tkinter
from tkinter import filedialog

root = tkinter.Tk()
root.withdraw()

Save = True
CSVpath = filedialog.askopenfilename(filetypes=[("CSVファイル", "tracks.csv")], title="読み込みたいtracks.csvを選択してください。")
path = "/".join(CSVpath.split("/")[:-1])

if path == "":
    exit()
if Save:
    DirName = path.split("/")[-1]
    #os.mkdir(DirName)
with open(CSVpath) as f:
    data = list(csv.reader(f))[1:]
data = [i[0].split("\t") for i in data]
D = {}
print(data)

for image, track_id, feature_id, x, y, _, r, g, b, _, _ in data:
    frame_id = image[:-4]
    track_id, feature_id, r, g, b = map(int, (track_id, feature_id, r, g, b))
    x, y = map(float, (x, y))
    if frame_id not in D:
        D[frame_id] = []
    D[frame_id].append([track_id, feature_id, x, y, r, g, b])
L = sorted(D.keys(), key=int)

pFL = {}

for frame_id in L:
    frame = cv2.imread(f"{path}/images/{frame_id}.jpg")
    height, width = frame.shape[:2]
    
    FL = {}
    for track_id, feature_id, x, y, r, g, b in D[frame_id]:
        pos = (int(width * (x + 0.5)), int(width * (y + 0.5) - (width - height) / 2))
        if track_id in pFL:
            cv2.line(frame, pos, pFL[track_id], ( 0, 0, 255), thickness=2)
            cv2.circle(frame, pos, 4, (  0,   0, 255), thickness=1, lineType=cv2.LINE_AA)
        else:
            cv2.circle(frame, pos, 4, ( 255, 255, 255), thickness=1, lineType=cv2.LINE_AA)
        FL[track_id] = pos
    pFL = FL
    cv2.imshow("result", frame)
    if Save:
        cv2.imwrite(f"./{DirName}/{frame_id}_features.jpg", frame)
    pres_key = cv2.waitKey(1) & 0xFF
    prop_val = cv2.getWindowProperty('result', cv2.WND_PROP_ASPECT_RATIO)
    if pres_key == ord('q') or prop_val < 0:
        exit()