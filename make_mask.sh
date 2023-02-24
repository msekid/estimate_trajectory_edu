#!/bin/sh
#source /home/sora-desktop/anaconda3/etc/profile.d/conda.sh
conda activate colmap0




dir_path="/home/sora-lab/dataset/data_horiba/*"
dirs=`find $dir_path -maxdepth 0 -type d`

for dir in $dirs;

do
    echo $dir
    cd $dir
    CURRENT=$(cd $(dirname $0);pwd)
    DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
    echo $DIR_NAME
    #cd /home/sora-desktop/OpenSfM
    #mkdir output
    #mkdir output/plotimage
    #ryuchi
    #python /home/sora-desktop/Documents/estimate_trajectory_smrc/save_data.py
    #sleep 3s
    #cd $dir
    #cp $dir/data.csv /home/sora-lab/Documents/unchi_toRyuchi/$DIR_NAME.csv

    #python /home/sora-lab/Documents/estimate_trajectory_smrc/main.py
    cd $dir
    #rm -rf image_0
    #mkdir image_0
    #python /home/sora-desktop/Documents/estimate_trajectory/trim.py
    mkdir masks 
    python /home/sora-lab/Documents/estimate_trajectory_smrc/create_mask.py 
    #rename 's/image_0/images/' $dir/image_0
    #cp /home/sora-desktop/dataset/camera_models_overrides.json $dir/camera_models_overrides.json
    #cp /home/sora-desktop/Documents/config_horiba.yaml $dir/config.yaml
    #cp /home/sora-desktop/Documents/camera_models_overrides_horiba.json $dir/camera_models_overrides.json
    #cp /home/sora-desktop/Documents/rename.py $dir/masks/rename.py
    #cp /home/sora-desktop/Documents/rename.sh $dir/masks/rename.sh
    #cd $dir/masks
    #source ./rename.sh
    #rm rename.sh
    #rm rename.py
    #cd $dir
    #rm -rf undistorted
    #cd /home/sora-desktop/OpenSfM
    #bin/opensfm_run_all $dir
done
cd /home/sora-lab/Documents/estimate_trajectory_smrc