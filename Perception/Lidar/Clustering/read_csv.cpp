#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
using namespace std;
// std::shared_ptr<geometry::PointCloud> read_csv(string filename)
// {
//   string delimiter = ",";
//   std::shared_ptr<geometry::PointCloud> point_cloud;
//   point_cloud=std::make_shared<geometry::PointCloud>();
//   //readfile
//   fstream file;
//   file.open(filename);
//   std::string line;
//   while (getline( file, line,'\n'))  
// 	{
// 	  istringstream templine(line); 
// 	  string data;

//     vector<string> v = split (line, delimiter);
//     Eigen::Vector3d d(stod(v[0]),stod(v[0]),stod(v[0]));
//     point_cloud->points_.push_back(d);

// 	}
//   file.close();
//   return point_cloud;


int main() {
    string fname = "lidar_ksi_30_3_23.csv";
    cout << fname << endl;
 
	vector<vector<string>> content;
	vector<string> row;
	string line, word;
 
	fstream file (fname, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
 
			stringstream str(line);
 
			while(getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);
		}
	}
	else
		cout<<"Could not open the file\n";
 
	for(int i=0;i<content.size();i++)
	{
		for(int j=0;j<content[i].size();j++)
		{
			cout<<content[i][j]<<" ";
		}
		cout<<"\n";
	}
 
	return 0;
}
//   