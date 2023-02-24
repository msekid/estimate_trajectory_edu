#!/bin/sh


dir_path="/mnt/source/dataset/DRDataset30/*"
dirs=`find $dir_path -maxdepth 0 -type d`

for dir in $dirs;
do
    echo $dir
    cd $dir
    CURRENT=$(cd $(dirname $0);pwd)
    DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
    echo $DIR_NAME
    cd $dir
    #cp /mnt/common/dataset/data_odometry_color/config.yaml $dir/config.yaml
    #cp /mnt/common/dataset/data_odometry_color/camera_models_overrides.json $dir/camera_models_overrides.json
    #cp -r $dir/image_2 $dir/images
done
cd /source/OpenSfM
for dir in $dirs;
do
    bin/opensfm_run_all $dir &
done
wait
