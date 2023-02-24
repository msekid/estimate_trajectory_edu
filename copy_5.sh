#!/bin/sh
#source /home/sora-desktop/anaconda3/etc/profile.d/conda.sh
conda activate smrc1




dir_path="/home/sora-lab/Pictures/takutosan/*"
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
    mkdir /home/sora-lab/dataset/takutosan/$DIR_NAME
    cp -r $dir /home/sora-lab/dataset/takutoan/$DIR_NAME/images
    cd /home/sora-lab/Documents/semantic-segmentation
    python -m torch.distributed.launch --nproc_per_node=1 train.py --dataset=SMRC --syncbn --apex --cv=0 --bs_val=1 --fp16 --is_smrc --eval=folder --eval_folder=$dir --n_scales="0.5,1.0,2.0" --dump_assets --dump_all_images --snapshot="/home/sora-lab/Documents/modified_semantic_segmentation/weights/2022_new_best_SMRC_trained.pth" --arch=ocrnet.HRNet_Mscale --result_dir=/home/sora-lab/dataset/takutosan/$DIR_NAME
    #cd $dir
    #rm -rf image_0
    #mkdir image_0
    #python /home/sora-desktop/Documents/estimate_trajectory/trim.py
    
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