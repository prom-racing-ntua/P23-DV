#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <vector>
#include <cstdint>
#include <Eigen/Core>
#include <fstream>
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
#include <nlohmann/json.hpp>
using namespace std;

namespace clustering {


struct cluster_output {
    std::vector<pcl::PointIndices> idx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cl;
};

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segm_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); //offical cluster object (to be returned)
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_init (new pcl::PointCloud<pcl::PointXYZ>), cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointIndices> cluster_indices;
//parameters for time execution
clock_t start1, end1, start2, end2, start3, end3;
// Read in the cloud data
std::vector<PointT> pcd_csv;
int cluster_count = 0;
double minX;
double maxX;
double minY;
double maxY;
double minZ;
double maxZ;
int maxIterations;
double DistanceThreshold;
int clusterMin;
int clusterMax;
double clusterTol;
double thres;
double hc;
double wc;
float a,b,c,d;

 std::vector<PointT> reader_csv(std::string name) {
    std::cout << "mpika reader" << std::endl;
    vector<vector<string>> content;
    std::vector<PointT> points;
    vector<string> row;
	string line, word;
    std::cout << "prin tin fstream" << std::endl;
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

void loadParams() {
    std::ifstream ifs("../config/params.json");
    nlohmann::json config = nlohmann::json::parse(ifs);

    minX = config["minX"];
    maxX = config["maxX"];

    minY = config["minY"];
    maxY = config["maxY"];

    minZ = config["minZ"];
    maxZ = config["maxZ"];

    maxIterations=config["MaxIterations"];
    DistanceThreshold=config["DistanceThreshold"];

    clusterMin=config["ClusterMin"];
    clusterMax=config["ClusterMax"];
    clusterTol=config["ClusterTol"];

    thres=config["Threshold"];
    hc=config["height_cone"];
    wc=config["width_cone"];
}

// compute the number of expected points for cone object
int num_expected_points(const pcl::PointXYZ &centre) {
    double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
    // static double hc = 0.30;               // cone height (or 29.8)
    // static double wc = 0.30;               // cone width (or 2*7.4=14.8 -> remains to be measured)
    static double rv = 0.0174533;  // angular resolution vertical at any freq (1° -> rad)
    static double rh = 0.003141593; // angular resolution horizontal at 10Hz(0.18° -> rad)
    static double rh2 = 0.006283185;  //angular resolution horizontal at 20Hz(0.36°  -> rad)
    static double rh3 = 0.001570796; //angular resolution horizontal at 5Hz(0.36°  -> rad)

    // compute and return number of expected points
    double E = 0.5 * hc / (2 * d * tan(rv / 2)) * wc / (2 * d * tan(rh / 2));
    return (int)E;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_box(pcl::PointCloud<pcl::PointXYZ>::Ptr var_cloud) {
    pcl::CropBox<pcl::PointXYZ> box;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    box.setInputCloud(var_cloud);
    box.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr var_cloud) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_temp (new pcl::PointCloud<pcl::PointXYZ>);
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations(maxIterations);
     seg.setDistanceThreshold(DistanceThreshold); //pio xamiilo xeirotero ground removal kai perissotera simeia sinolo
     std::cout << "dist thres is: " << DistanceThreshold << std::endl; 
    //  int nr_points = (int) cloud_filtered->size ();
    //  while (cloud_filtered->size () > 0.3 * nr_points)
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (var_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZ> extract;
       extract.setInputCloud (cloud_box_out);
       extract.setIndices (inliers);
       extract.setNegative (false);
       a = coefficients->values[0];
       b = coefficients->values[1];
       c = coefficients->values[2];
       d = coefficients->values[3];
       // Get the points associated with the planar surface
       extract.filter (*cloud_plane);
       // Remove the planar inliers, extract the rest
       extract.setNegative (true);
       extract.filter (*cloud_out_temp);
       return cloud_out_temp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr var_cloud) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_temp (new pcl::PointCloud<pcl::PointXYZ>);
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations(maxIterations);
     seg.setDistanceThreshold(DistanceThreshold); //pio xamiilo xeirotero ground removal kai perissotera simeia sinolo
     std::cout << "dist thres is: " << DistanceThreshold << std::endl; 
    //  int nr_points = (int) cloud_filtered->size ();
    //  while (cloud_filtered->size () > 0.3 * nr_points)
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (var_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZ> extract;
       extract.setInputCloud (cloud_box_out);
       extract.setIndices (inliers);
       extract.setNegative (false);
       a = coefficients->values[0];
       b = coefficients->values[1];
       c = coefficients->values[2];
       d = coefficients->values[3];
       // Get the points associated with the planar surface
       extract.filter (*cloud_plane);
       // Remove the planar inliers, extract the rest
       extract.setNegative (true);
       extract.filter (*cloud_out_temp);
       return cloud_out_temp;
}

cluster_output clustering_pipeline(pcl::PointCloud<pcl::PointXYZ>::Ptr var_cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    //faster method 
    FastEuclideanClustering<pcl::PointXYZ> fec;
    fec.setInputCloud(var_cloud);
    fec.setSearchMethod(tree);
    fec.setMinClusterSize(clusterMin);
    fec.setMaxClusterSize(clusterMax);
    fec.setClusterTolerance(clusterTol); //itan 0.08 
    fec.setQuality(0.0);
    fec.segment (cluster_indices);

    return {cluster_indices,var_cloud};
}

void CsvToPcd(std::vector<PointT> input_){
    PointCloudT::Ptr cloud_temp(new PointCloudT);
    cloud_temp->width = input_.size();
    cloud_temp->height = 1;
    cloud_temp->points.resize(cloud_temp->width * cloud_temp->height);
    for (std::size_t i = 0; i < input_.size(); ++i) {
        cloud_temp->points[i] = input_[i];
    }
    //write to point cloud
    pcl::io::savePCDFileBinary("../DataKsi2/pcd/cloud_cluster_init.pcd", *cloud_temp);
    std::cout << "Saved " << cloud_temp->size() << " data points to file: " << "cloud_cluster_init.pcd" << std::endl;
}
}//end namespace