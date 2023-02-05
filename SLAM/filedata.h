#ifndef FILEDATA_H
#define FILEDATA_H

#include <fstream>
#include <string>
#include <iostream>
#include <limits>

class read_file {
public:
  read_file() {
    perception->timestamp = 0;
    perception->color = 0;
    perception->range = 0;
    perception->theta = 0;

    odometry->timestamp = 0;
    odometry->xvelocity = 0;
    odometry->yvelocity = 0;
    odometry->omega = 0;
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
        odometry->cov_mat[i][j] = 0;
      }
    }
  }

  ~read_file() {
    delete perception;
    delete odometry;
  }

  std::fstream& GotoLine(std::fstream& file, int num){
      file.seekg(std::ios::beg);
      for(int i=0; i < num - 1; ++i){
          file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      }
      return file;
  }

  bool read_odometry(int odom_num){
    std::fstream file;
    file.open("/home/user/gtsam/examples/AutoX-3-Velocity.txt",std::fstream::in);
    if (file.is_open()) {   //checking whether the file is open
      this->GotoLine( file, odom_num);
      file >> odometry->timestamp;
      file >> odometry->xvelocity;
      file >> odometry->yvelocity;
      file >> odometry->omega;
      int i=0;
      int j=0;
      for(int iter=0; iter<9; iter++) {
        file >> odometry->cov_mat[i][j%3];
        j++;
        if (j%3 == 0) i++;
      }
    }
    if (file.eof()) return 1;
    file.close();
    return 0;
  }

  bool read_perception(int per_num){
    std::fstream file;
    file.open("/home/user/gtsam/examples/AutoX-3-Perception.txt",std::fstream::in);
    if (file.is_open()) {   //checking whether the file is open
      this->GotoLine(file, per_num);
      file >> perception->timestamp;
      file >> perception->color;
      file >> perception->range;
      file >> perception->theta;
    }
    if (file.eof()) return 1;
    file.close();
    return 0;
  }



  struct perception_measurement{
    int timestamp;
    int color;
    double range;
    double theta;
  };
  struct perception_measurement* perception = new perception_measurement;

  struct odometry_measurement{
    int timestamp;
    double xvelocity;
    double yvelocity;
    double omega;
    double cov_mat[3][3];
  };
  struct odometry_measurement* odometry = new odometry_measurement;

};

#endif // FILEDATA_H
