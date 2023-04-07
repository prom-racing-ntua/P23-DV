# Lidar Segmentation and Clustering Pipeline

This folder includes the main pipeline for cone cluster extraction. It requires Ubuntu 20.04 and PCL Library (Version 1.13)

# PCL 1.13 Installation

```
git clone https://github.com/PointCloudLibrary/pcl.git
mkdir build && cd build
cmake ..
make -j2 && sudo make install
```

# Code Execution

```
cd P23-DV/Perception/Lidar/Clustering
cmake CMakeLists.txt
make
./lidar_cluster_cpp
```

# Result visualization after each step

```
sudo apt-get install -y pcl-tools
pcl_viewer cloud_cluster_ksi.pcd
pcl_viewer cloud_cluster_segm.pcd
pcl_viewer cloud_cluster_final.pcd
```
