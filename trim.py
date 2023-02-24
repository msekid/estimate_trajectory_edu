

# -*- coding: utf-8 -*-
import sys
from PIL import Image
import os

#トリミング前の画像の格納先
ORIGINAL_FILE_DIR = "images/"
#トリミング後の画像の格納先
TRIMMED_FILE_DIR = "image_0/"

#画像パスと、左上座標、右下座標を指定して、トリミングされたimageオブジェクトを返す。
def trim(path, left, top, right, bottom):
  im = Image.open(path)
  im_trimmed = im.crop((left,top,right,bottom))
  return im_trimmed


if __name__ == '__main__':
  #もしトリミング後の画像の格納先が存在しなければ作る
  if os.path.isdir(TRIMMED_FILE_DIR) == False:
    os.makedirs(TRIMMED_FILE_DIR)

  #トリミングする左上の座標
  left, top = 0, 0
  #トリミングする右上の座標
  right, bottom = 1280, 510

  #画像ファイル名を取得
  files = os.listdir(ORIGINAL_FILE_DIR)
  #特定の拡張子のファイルだけを採用。実際に加工するファイルの拡張子に合わせる
  files = [name for name in files if name.split(".")[-1] in ["png","jpg"]]

  for val in files:
    #オリジナル画像へのパス
    path = ORIGINAL_FILE_DIR + val
    #トリミングされたimageオブジェクトを取得
    im_trimmed = trim(path, left, top, right, bottom)
    #トリミング後のディレクトリに保存。ファイル名の頭に"cut_"をつけている
    im_trimmed.save(TRIMMED_FILE_DIR+val, quality=95) #qualityは95より大きい値は推奨されていないらしい