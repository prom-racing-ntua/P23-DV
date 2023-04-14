#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <vector>
#include <cstdint>
#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointField.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <stdio.h>
#include <iomanip>
#include "fast_euclidean_clustering.h"

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
            // point.I = std::stof(row[])
            points.push_back(point);
		}
	}
	else
		cout<<"Could not open the file\n";
    return points;
}

// compute the number of expected points for cone object
int num_expected_points(const pcl::PointXYZ &centre) {
    double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
    static double hc = 0.298;               // cone height (or 29.8)
    static double wc = 0.30;               // cone width (or 2*7.4=14.8 -> remains to be measured)
    static double rv = 0.0174533;  // angular resolution vertical at any freq (1° -> rad)
    static double rh = 0.003141593; // angular resolution horizontal at 10Hz(0.18° -> rad)
    static double rh2 = 0.006283185;  //angular resolution horizontal at 20Hz(0.36°  -> rad)
    static double rh3 = 0.001570796; //angular resolution horizontal at 5Hz(0.36°  -> rad)

    // compute and return number of expected points
    double E = 0.5 * hc / (2 * d * tan(rv / 2)) * wc / (2 * d * tan(rh / 2));
    return (int)E;
}

int main () {
    // pcl::uindex_t index = 0;
    std::cout << PCL_VERSION << std::endl; 
     //parameters for time execution
     clock_t start1, end1, start2, end2;
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

     start1 = clock(); //before lidar pipeline
    
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
       end1 = clock(); //before lidar pipeline
     
    writer.write<pcl::PointXYZ> ("cloud_cluster_segm.pcd", *cloud_filtered, false); //*
    
     // Creating the KdTree object for the search method of the extraction
    //  start2 = clock(); //before lidar pipeline
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud_filtered);
     std::vector<pcl::PointIndices> cluster_indices;

     //first method
    //  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //  ec.setClusterTolerance(0.5); //parapanw simeia oso to auksanw (why tho??)
    //  ec.setMinClusterSize(5);
    //  ec.setMaxClusterSize(250);
    //  ec.setSearchMethod (tree);
    //  ec.setInputCloud (cloud_filtered);
    //  start2 = clock(); //before lidar pipeline
    //  ec.extract (cluster_indices);
    //  end2 = clock(); //before lidar pipeline

     //faster method
     start2 = clock(); //before lidar pipeline
      FastEuclideanClustering<pcl::PointXYZ> fec;
      fec.setInputCloud(cloud_filtered);
      fec.setSearchMethod(tree);
      fec.setMinClusterSize(5);
      fec.setMaxClusterSize(250);
      fec.setClusterTolerance(0.5);
      fec.setQuality(0.0);
      fec.segment (cluster_indices);
      end2 = clock(); //before lidar pipeline
   
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); //offical cluster object (to be returned)
     for (const auto& cluster : cluster_indices) {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZ>); 
       for (const auto& idx : cluster.indices) {
         cloud_cluster_temp->push_back((*cloud_filtered)[idx]);
        //  cloud_cluster->push_back((*cloud_filtered)[idx]);
       }

      // extract centroid of cluster
      pcl::PointXYZ centre;
      pcl::computeCentroid(*cloud_cluster_temp, centre); 

      //whatever
      double d = std::sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
      int expected = num_expected_points(centre);
      std::cout << "num_expected_points = " << expected << std::endl;
      std::cout << "num actual points   = " << cloud_cluster_temp->size() << std::endl;
      std::cout << "distance to cone    = " << d << std::endl;

      double thres = 0.5; //percentage threshold in order to determine relevance of lidar cluster to cone cluster
      if ((cloud_cluster_temp->size() > (1-thres)*expected) && (cloud_cluster_temp->size() < (1+thres)*expected)) {
          for (const auto& idx : cluster.indices)
            cloud_cluster->push_back((*cloud_filtered)[idx]);
          std::cout << "i kept that sh*t" << std::endl;
          std::cout << "centre of cluster is: " << centre.x << " " << centre.y << " " << centre.z << std::endl;
      }
      else{
        std::cout << "i left that sh*t" << std::endl;
        std::cout << " " << std::endl;
        continue;
      }
     }

    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    end2 = clock(); //before lidar pipeline
    // end2 = clock();
    double time_taken_segm = double(end1 - start1) / double(CLOCKS_PER_SEC);
    double time_taken_clust = double(end2 - start2) / double(CLOCKS_PER_SEC);
    cout << "Time taken for segm is : " << fixed
         << time_taken_segm << std::setprecision(5);
    cout << " secs " << endl;
    std::cout << " " << std::endl;
    cout << "Time taken for clust is : " << fixed
         << time_taken_clust << std::setprecision(5);
    cout << " secs " << endl;
    std::cout << " " << std::endl;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    // std::stringstream ss;
    // ss << std::setw(4) << std::setfill('0') << j;
    writer.write<pcl::PointXYZ> ("cloud_cluster_final.pcd", *cloud_cluster, false); //*
  return (0);
}