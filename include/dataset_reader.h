//
// Created by zyd on 23-8-10.
//

#ifndef CORNERVINS_DATASET_READER_H
#define CORNERVINS_DATASET_READER_H

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace Rvg {


    class CIDDatasetReader
    {

    public:

        static void LoadImageDataCID(const std::string dataset_path, std::vector<std::pair<double, std::string>>& image_data)
        {
            std::string path = dataset_path + "associations.txt";
            std::ifstream fin;
            fin.open(path);
            if(!fin.is_open())
            {
                std::cerr << "------------ Reading image data error --------------" << std::endl;
                return;
            }

            while(!fin.eof())
            {
                std::string str;
                getline(fin, str);

                if(!str.empty())
                {
                    std::stringstream ss;
                    ss << str;

                    std::string timestamp_str;
                    ss >> timestamp_str;
                    std::string img_name = timestamp_str;

                    double timestamp = atof(timestamp_str.c_str());
                    timestamp = timestamp;  // unit: s

                    image_data.push_back(std::make_pair(timestamp, img_name));
                }
            }

            fin.close();
        }

        static void LoadImuDataCID(const std::string dataset_path, std::vector<Eigen::Matrix<double, 7, 1>>& imu_data)
        {
            std::string path_imu = dataset_path + "imu.txt";
            std::ifstream fin;
            fin.open(path_imu);
            if(!fin.is_open())
            {
                std::cerr << "------------ Reading IMU data error --------------" << std::endl;
                return;
            }

            while(!fin.eof())
            {
                std::string str;
                getline(fin, str);

                if(!str.empty())
                {
                    std::stringstream ss;
                    ss << str;

                    double timestamp, wx, wy, wz, ax, ay, az;
                    ss >> timestamp;
                    ss >> wx; ss >> wy; ss >> wz;
                    ss >> ax; ss >> ay; ss >> az;

                    Eigen::Matrix<double, 7, 1> data;
                    data << timestamp, wx, wy, wz, ax, ay, az;
                    imu_data.push_back(data);
                }
            }

            fin.close();
        }

    private:

        CIDDatasetReader() {}

    };

}

#endif //CORNERVINS_DATASET_READER_H
