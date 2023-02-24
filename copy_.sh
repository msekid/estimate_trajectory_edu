#!/bin/sh
#source /home/sora-desktop/anaconda3/etc/profile.d/conda.sh
conda activate est




dir_path="/Users/sora-mac/dataset/DRDataset30/*"
dirs=`find $dir_path -maxdepth 0 -type d`

for dir in $dirs;
do
    echo $dir
    cd $dir
    CURRENT=$(cd $(dirname $0);pwd)
    DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
    str=`echo ${dir} | awk -F "/" '{ print $NF }'`
    echo ${str}
    echo $DIR_NAME
    #cd /home/sora-desktop/OpenSfM
    #mkdir output
    #mkdir output/plotimage
    #ryuchi
    #python /home/sora-lab/Documents/estimate_trajectory_smrc/save_data.py
    #sleep 3s
    #cd $dir
    #cp $dir/data.csv /home/sora-lab/Documents/unchi_toRyuchi/$DIR_NAME.csv

    cd $dir
    #rm -rf image_0
    #mkdir image_0
    #mkdir png
    #python /Users/sora-mac/Documents/estimate_trajectory_smrc/save_data.py &
    cp $dir/output/map_svd.html /Users/sora-mac/Pictures/map_svd/$str.html
    
    #python /home/sora-desktop/Documents/estimate_trajectory/create_mask.py 
    #rename 's/image_0/images/' $dir/image_0
    #cp /home/sora-desktop/dataset/camera_models_overrides.json $dir/camera_models_overrides.json
    #cp /home/sora-desktop/Documents/config.yaml $dir/config.yaml
    #cp /home/sora-desktop/Documents/camera_models_overrides.json $dir/camera_models_overrides.json
    #cp /home/sora-desktop/Documents/rename.py $dir/masks/rename.py
    #cp /home/sora-desktop/Documents/rename.sh $dir/masks/rename.sh
    #cd $dir/masks
    #source ./rename.sh
    #rm rename.sh
    #rm rename.py
    #cd $dir
    #rm -rf undistorted
    #bin/opensfm_run_all $dir
done
#wait