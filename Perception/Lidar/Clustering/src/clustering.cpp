#include "clustering.h"
using json = nlohmann::json;
using namespace std;
using namespace clustering;

int main () {
    // pcl::uindex_t index = 0;
    std::cout << "pcl version is: " <<PCL_VERSION << std::endl; 
    
    pcd_csv=reader_csv("../DataKsi2/lidar/datastatic_lidar.csv");
    std::cout << "read csv" << std::endl; 

    loadParams();
    std::cout << "loaded params" << std::endl; 
    CsvToPcd(pcd_csv);

     pcl::PCDReader reader;
     reader.read ("../DataKsi2/pcd/cloud_cluster_init.pcd", *cloud_init);
     std::cout << "PointCloud before filtering has: " << cloud_init->size () << " data points." << std::endl; 

    // crop box filtering
     start1 = clock(); 
     cloud_box_out=crop_box(cloud_init);
     end1 = clock();
     pcl::PCDWriter writer;
     std::cout << "PointCloud after crop box filtering has: " << cloud_box_out->size ()  << " data points." << std::endl; //*
     writer.write<pcl::PointXYZ> ("../DataKsi2/pcd/cloud_cluster_box.pcd", *cloud_box_out, false); //*

     //segmentation
     start2=clock();
     cloud_segm_out=segmentation(cloud_box_out);
     end2=clock();
     std::cout << "coeffs of planar are: " << a << " " << b  << " " << c << " " << d << std::endl;
     std::cout << "PointCloud after segmentation has: " << cloud_segm_out->size ()  << " data points." << std::endl;
     writer.write<pcl::PointXYZ> ("../DataKsi2/pcd/cloud_cluster_segm.pcd", *cloud_segm_out, false); //*

     //clustering
     start3 = clock();
     cluster_output c_o = cluster_output();
     c_o = clustering_pipeline(cloud_segm_out);
     cluster_indices = c_o.idx;
     cloud_final = c_o.cl;
     end3 = clock(); 
     
     ofstream outputFile("../DataKsi2/output_clustering.csv");
     outputFile << "X,Y,Z\n";
    
     for (const auto& cluster : cluster_indices) {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZ>); 
       for (const auto& idx : cluster.indices) {
         cloud_cluster_temp->push_back((*cloud_final)[idx]);
       }

      // extract centroid of cluster
      pcl::PointXYZ centre;
      pcl::computeCentroid(*cloud_cluster_temp, centre); 

      double d = std::sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
      int expected = num_expected_points(centre);
      std::cout << "num_expected_points = " << expected << std::endl;
      std::cout << "num actual points   = " << cloud_cluster_temp->size() << " at distance " << d << std::endl;
      if ((cloud_cluster_temp->size() > (1-thres)*expected) && (cloud_cluster_temp->size() < (1+thres)*expected)) {
          std::cout << "i kept that sh*t" << std::endl;
          cluster_count++;
          // Find the four points with the minimum and maximum X and Z coordinates
          pcl::PointXYZ minX = cloud_cluster_temp->points[0], maxX = cloud_cluster_temp->points[0], minZ = cloud_cluster_temp->points[0], maxZ = cloud_cluster_temp->points[0];
          for (int i = 1; i < cloud_cluster_temp->points.size(); i++) {
              if (cloud_cluster_temp->points[i].x < minX.x) {
                  minX = cloud_cluster_temp->points[i];
              }
              if (cloud_cluster_temp->points[i].x > maxX.x) {
                  maxX = cloud_cluster_temp->points[i];
              }
              if (cloud_cluster_temp->points[i].z < minZ.z) {
                  minZ = cloud_cluster_temp->points[i];
              }
              if (cloud_cluster_temp->points[i].z > maxZ.z) {
                  maxZ = cloud_cluster_temp->points[i];
              }
          }
          outputFile << "--,cluster"<<cluster_count<<",--\n";
          outputFile << centre.x << "," << centre.y << "," << centre.z << "\n";
          outputFile << minX.x << "," << minX.y << "," << minX.z << "\n";
          outputFile << maxX.x << "," << maxX.y << "," << maxX.z << "\n";
          outputFile << minZ.x << "," << minZ.y << "," << minZ.z << "\n";
          outputFile << maxZ.x << "," << maxZ.y << "," << maxZ.z << "\n";
          for (const auto& idx : cluster.indices)
            cloud_cluster->push_back((*cloud_final)[idx]); //for visualization
          std::cout << "centre of cluster is: " << centre.x << "," << centre.y << "," << centre.z << std::endl;
          std::cout << "distance from cone is: " << d << std::endl;
          std::cout << " " << std::endl;
      }
      else{
        std::cout << "i left that sh*t" << std::endl;
        std::cout << " " << std::endl;
        continue;
      }
     }
    outputFile.close();
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    double time_taken_box = double(end1 - start1) / double(CLOCKS_PER_SEC);
    double time_taken_segm = double(end2 - start2) / double(CLOCKS_PER_SEC);
    double time_taken_clust = double(end3 - start3) / double(CLOCKS_PER_SEC);
    cout << "Time taken for filtering is : " << fixed
         << time_taken_box << std::setprecision(5);
    cout << " secs " << endl;
    std::cout << " " << std::endl;
    cout << "Time taken for segm is : " << fixed
         << time_taken_segm << std::setprecision(5);
    cout << " secs " << endl;
    std::cout << " " << std::endl;
    cout << "Time taken for clust is : " << fixed
         << time_taken_clust << std::setprecision(5);
    cout << " secs " << endl;
    std::cout << " " << std::endl;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::cout << "Number of clusters: " << cluster_count << std::endl;
    // std::stringstream ss;
    // ss << std::setw(4) << std::setfill('0') << j;
    writer.write<pcl::PointXYZ> ("../DataKsi2/pcd/cloud_cluster_final.pcd", *cloud_cluster, false); //*
    return (0);
}