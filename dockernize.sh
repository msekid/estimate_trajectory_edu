#!/bin/sh

docker exec -it estraj bash
cd /ORB_SLAM3
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/denso.yaml /mnt/source/dataset/DRDataset30/564955