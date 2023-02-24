#!/bin/sh
#source /home/sora-desktop/anaconda3/etc/profile.d/conda.sh
conda activate colmap0



#cd /home/sora-desktop/ORB_SLAM3
dir_path="/home/sora-desktop/dataset/DRDataset30/*"
dirs=`find $dir_path -maxdepth 0 -type d`
for i in `seq 0 9`;
do
    for dir in $dirs;
    do
        #conda activate droidenv
        echo $dir
        #cd $dir
        #python /home/sora-desktop/Documents/estimate_trajectory_smrc/make_timelist.py
        
        #cd $dir
        #python /home/sora-lab/Documents/estimate_trajectory_smrc/temp.py
        #cd $dir
        #rm -rf image_0
        #rm -rf reports
        #rm -rf exif
        #rm -rf masks
        #rm -rf matches
        #rm -rf pred_mask
        #rm -rf undistorted
        #rm -rf features
        #mkdir image_0
        
        #python /home/sora-desktop/createmap/create_mask.py 
        #rename 's/image_0/images/' $dir/image_0
        #cp /home/sora-desktop/dataset/times.txt $dir/times.txt
        #cp /home/sora-desktop/dataset/camera_models_overrides.json $dir/camera_models_overrides.json
        #cp /home/sora-desktop/dataset/config.yaml $dir/config.yaml
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
        cd $dir
        mkdir $dir/OUTPUT_ORBSLAM
        python /home/sora-desktop/Documents/estimate_trajectory_smrc/make_timelist.py
        cd /home/sora-desktop/ORB_SLAM3
        ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/denso.yaml $dir
        cp /home/sora-desktop/ORB_SLAM3/KeyFrameTrajectory.txt $dir/OUTPUT_ORBSLAM/$i.txt
        #cd /home/sora-desktop/DROID-SLAM
        #python demo.py --imagedir=$dir/image_0 --calib=calib/denso.txt --reconstruction_path=smrc --image_size=[510,1280] --stride=1 --disable_vis
        #cp /home/sora-desktop/DROID-SLAM/reconstructions/smrc/tstamps.npy $dir/tstamps.npy
        #cp /home/sora-desktop/DROID-SLAM/reconstructions/smrc/poses.npy $dir/poses.npy
    done
done
