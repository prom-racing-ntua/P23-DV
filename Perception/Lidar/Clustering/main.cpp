#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

 std::vector<PointT> reader_csv(string name) {
    vector<vector<string>> content;
    std::vector<PointT> points;
    vector<string> row;
	string line, word;
 
	fstream file (name, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
 
			stringstream str(line);
 
			while(getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);

            PointT point;
            point.x = std::stof(row[7]);
            point.y = std::stof(row[8]);
            point.z = std::stof(row[9]);
            points.push_back(point);
		}
	}
	else
		cout<<"Could not open the file\n";
    // for(int i=0;i<content.size();i++)
	// {
	// 	for(int j=0;j<content[i].size();j++)
	// 	{
	// 		cout<<content[i][j]<<" ";
	// 	}
	// 	cout<<"\n";
	// }
    return points;
}

int main () {
     // Read in the cloud data
     std::vector<PointT> pcd_csv;
     pcd_csv=reader_csv("lidar_ksi_30_3_23.csv");

    //try to write pcd
    PointCloudT::Ptr cloud_temp(new PointCloudT);
    cloud_temp->width = pcd_csv.size();
    cloud_temp->height = 1;
    // cloud->is_dense = false;
    cloud_temp->points.resize(cloud_temp->width * cloud_temp->height);

    for (std::size_t i = 0; i < pcd_csv.size(); ++i) {
        cloud_temp->points[i] = pcd_csv[i];
    }

     //write to point cloud
     pcl::io::savePCDFileBinary("cloud_cluster_ksi.pcd", *cloud_temp);
     std::cout << "Saved " << cloud_temp->size() << " data points to file: " << "cloud_cluster_ksi.pcd" << std::endl;

     pcl::PCDReader reader;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
     reader.read ("cloud_cluster_ksi.pcd", *cloud);
     std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; 
    
    double minX = -5.0;
	double maxX = 5.0;

	double minY = -15.0;
	double maxY = 0.0;

	double minZ = -5.0;
	double maxZ = 5.0;

	// perform crop box filtering
	pcl::CropBox<pcl::PointXYZ> box;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	box.setInputCloud(cloud);
	box.filter(*cloud_filtered);
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
    //  pcl::VoxelGrid<pcl::PointXYZ> vg;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //  vg.setInputCloud (cloud);
    //  vg.setLeafSize (0.01f, 0.01f, 0.01f);
    //  vg.filter (*cloud_filtered);
     std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
   
     // Create the segmentation object for the planar model and set all the parameters
     pcl::SACSegmentation<pcl::PointXYZ> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PCDWriter writer;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (1000);
     seg.setDistanceThreshold(0.03); //pio xamiilo xeirotero ground removal kai perissotera simeia sinolo
   
    //  int nr_points = (int) cloud_filtered->size ();
    //  while (cloud_filtered->size () > 0.3 * nr_points)
       // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZ> extract;
       extract.setInputCloud (cloud_filtered);
       extract.setIndices (inliers);
       extract.setNegative (false);
       float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];

        std::cout << "coeffs of planar are: " << a << " " << b  << " " << c << " " << d << std::endl;
   
       // Get the points associated with the planar surface
       extract.filter (*cloud_plane);
       std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
   
       // Remove the planar inliers, extract the rest
       extract.setNegative (true);
       extract.filter (*cloud_f);
       *cloud_filtered = *cloud_f;
     
    writer.write<pcl::PointXYZ> ("cloud_cluster_segm.pcd", *cloud_filtered, false); //*
    
     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud_filtered);
   
     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
     ec.setClusterTolerance(0.5); //parapanw simeia oso to auksanw (why tho??)
     ec.setMinClusterSize(5);
     ec.setMaxClusterSize(250);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);
   
     int j = 0;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     for (const auto& cluster : cluster_indices)
     {
       for (const auto& idx : cluster.indices) {
         cloud_cluster->push_back((*cloud_filtered)[idx]);
       }
     } 
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    // std::stringstream ss;
    // ss << std::setw(4) << std::setfill('0') << j;
    writer.write<pcl::PointXYZ> ("cloud_cluster_final.pcd", *cloud_cluster, false); //*
    j++; 
  return (0);
}