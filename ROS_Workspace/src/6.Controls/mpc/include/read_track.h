#ifndef READ_TRACK_H
#define READ_TRACK_H

#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <eigen3/Eigen/Core>


inline Eigen::MatrixXd readTrack(const std::string& filename) {
    int cols{ 0 };
    int rows{ 0 };
    std::array<double, (int)1e6> buffer{};
    std::string line;

    // Read numbers from the file into the buffer
    std::ifstream data_file;
    data_file.open(filename);
    while (!data_file.eof())
    {
        std::getline(data_file, line);

        int temp_cols{ 0 };
        std::stringstream stream(line);
        while (!stream.eof())
            stream >> buffer[cols * rows + temp_cols++];

        line.clear();
        if (temp_cols == 0) continue;
        if (cols == 0) cols = temp_cols;
        rows++;
    }
    data_file.close();
    rows--;
    // Populate the eigen matrix with the numbers from the buffer
    std::cout << "rows and cols on read_track are: " << rows << " " << cols << std::endl;
    Eigen::MatrixXd result{ rows, cols };
    for (int i{ 0 }; i < rows; i++)
        for (int j{ 0 }; j < cols; j++) {
            result(i, j) = buffer[cols * i + j];
            std::cout << "i, j and result is: " << i << " " << j << " " << buffer[cols*i+j] << std::endl;
    }
    return result;
}


// int main() {
//     std::string path{"/home/dimitris/Prom_Driverless/draft_code/path_planning/test_tracks/trackdrive_midpoints.txt"};
//     Eigen::MatrixXd midpoints{ readTrack(path) };

//     std::cout << midpoints << '\n';

//     return 0;
// }

#endif