#!/bin/sh
#source /home/sora-desktop/anaconda3/etc/profile.d/conda.sh
conda activate colmap0




dir_path="/home/sora-lab/dataset/DRDataset30/*"
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
    #python /home/sora-lab/Documents/estimate_trajectory_smrc/save_data.py
    #sleep 3s
    #cd $dir
    #cp $dir/data.csv /home/sora-lab/Documents/unchi_toRyuchi/$DIR_NAME.csv

    cd $dir
    #mkdir $dir/output
    #mkdir $dir/output/opted
    #rm -rf image_0
    #mkdir image_0
    #mkdir png
    rm -rf best_images
    rm -rf color_blend
    rm -rf exif
    rm -rf reports
    rm -rf matches
    rm -rf features
    rm -rf undistorted

    #python /home/sora-lab/Documents/estimate_trajectory_smrc/save_extractedRoads.py
    
    
    
    python /home/sora-lab/Documents/estimate_trajectory_smrc/main.py
    python /home/sora-lab/Documents/estimate_trajectory_smrc/save_data.py
    sleep 5s
    python /home/sora-lab/Documents/estimate_trajectory_smrc/create_animemap.py
    sleep 5s
    python /home/sora-lab/Documents/estimate_trajectory_smrc/make_videos.py
    sleep 5s
    cp $dir/final_output.csv /home/sora-lab/Desktop/output_tabata/csv/$DIR_NAME.csv
    cp $dir/output/map_final.html /home/sora-lab/Desktop/output_tabata/html/$DIR_NAME.html
    cp -r $dir/output/opted /home/sora-lab/Desktop/output_tabata/png/$DIR_NAME
    cp $dir/original.mp4 /home/sora-lab/Desktop/output_tabata/mp4/$DIR_NAME.mp4
    
    rm -rf /home/sora-lab/Desktop/output_tabata/png/$DIR_NAME/opted
    
    cp $dir/final_output_orb.csv /home/sora-lab/Desktop/output_tabata/csv_orb/$DIR_NAME.csv
    cp $dir/final_output_sfm.csv /home/sora-lab/Desktop/output_tabata/csv_sfm/$DIR_NAME.csv
    
    #python /home/sora-lab/Documents/estimate_trajectory_smrc/create_animemap.py
    cp $dir/data.csv /home/sora-lab/Documents/DRDataset30_toRyuchi/$DIR_NAME.csv
    #cp -r output /home/sora-lab/Pictures/DRDataset30_output/$DIR_NAME
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