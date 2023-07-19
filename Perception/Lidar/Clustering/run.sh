CURRENT_DIR=$(pwd)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/build
cmake ..
make
./lidar_cluster_cpp
cd ..
# pcl_viewer DataKsi2/pcd/cloud_cluster_init.pcd
python3 projection.py 
pcl_viewer DataKsi2/pcd/cloud_cluster_final.pcd