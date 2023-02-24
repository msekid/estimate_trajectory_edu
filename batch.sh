#!/bin/sh

conda activate colmap0
dir_path="/home/sora-lab/dataset/data_horiba/*"
dirs=`find $dir_path -maxdepth 0 -type d`
for dir in $dirs;
do
    cd $dir
    CURRENT=$(cd $(dirname $0);pwd)
    DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
    echo $DIR_NAME
    #cp -r /home/sora-lab/dataset/unchi/images/$DIR_NAME $dir/images
    S=$(printf "%08d\n" "${DIR_NAME}")
    echo "$S"
    #cp /home/sora-lab/dataset/unchi/csv_files/ID${S}.csv $dir/$DIR_NAME.csv
    cd $dir
    cp reconstruction.json /home/sora-lab/Desktop/unchi_reconstruction/$DIR_NAME.json

    #segmentation
    #conda activate smrc1
    #cd /home/sora-lab/Documents/semantic-segmentation
    #python -m torch.distributed.launch --nproc_per_node=1 train.py --dataset=SMRC --syncbn --apex --cv=0 --bs_val=1 --fp16 --is_smrc --eval=folder --eval_folder=$dir/images --n_scales="0.5,1.0,2.0" --dump_assets --dump_all_images --snapshot="/home/sora-lab/Documents/modified_semantic_segmentation/weights/2022_new_best_SMRC_trained.pth" --arch=ocrnet.HRNet_Mscale --result_dir=$dir
    

    #mkdir /home/sora-lab/dataset/data_horiba/$DIR_NAME
done

cd ~/Documents/estimate_trajectory_smrc