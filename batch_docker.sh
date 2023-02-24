conda activate colmap0




dir_path="/mnt/source/dataset/DRDataset30/*"
dirs=`find $dir_path -maxdepth 0 -type d`

for i in `seq 0 9`;
do
    for dir in $dirs;
    do
        echo $dir
        cd $dir
        cd /ORB_SLAM3
        ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/denso.yaml $dir
        cp /ORB_SLAM3/KeyFrameTrajectory.txt $dir/OUTPUT_ORBSLAM/$i.txt

    done
done
