conda activate colmap0


dir_path="/home/sora-lab/dataset/data_horiba/*"
dirs=`find $dir_path -maxdepth 0 -type d`
for dir in $dirs;
do
    cd $dir
    CURRENT=$(cd $(dirname $0);pwd)
    DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
    echo $DIR_NAME
    cd /home/sora-lab/dataset/data_horiba
    mkdir $dir
    cd $dir
    python /home/sora-lab/Documents/estimate_trajectory_smrc/get_camera_param.py $DIR_NAME


done

