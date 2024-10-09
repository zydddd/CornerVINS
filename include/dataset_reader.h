//
// Created by zyd on 23-8-10.
//

#ifndef RGBD_PLANE_VIO_DATASET_READER_H
#define RGBD_PLANE_VIO_DATASET_READER_H

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace RvgVio {

/**
 * @brief Helper functions to read in dataset files
 */
    class DatasetReader
    {

    public:
        static void LoadImageDataETH(const std::string dataset_path, std::vector<std::pair<double, std::string>>& image_data)
        {
            std::string path = dataset_path + "associated.txt";
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

                    double timestamp;
                    ss >> timestamp;
                    std::string img_name;
                    ss >> img_name;
                    image_data.push_back(std::make_pair(timestamp, img_name));
                }
            }

            fin.close();
        }

        static void LoadDepthDataETH(const std::string dataset_path, std::vector<std::pair<double, std::string>>& image_data)
        {
            std::string path = dataset_path + "associated.txt";
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

                    double timestamp;
                    ss >> timestamp;
                    std::string img_name;
                    ss >> img_name;

                    ss >> timestamp;
                    ss >> img_name;
                    image_data.push_back(std::make_pair(timestamp, img_name));
                }
            }

            fin.close();
        }

        static void LoadImuDataETH(const std::string dataset_path, std::vector<Eigen::Matrix<double, 7, 1>>& imu_data)
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

        static void LoadImageDataWithGtCID(const std::string dataset_path,
                                           std::vector<std::pair<double, std::string>>& image_data,
                                           std::vector<std::pair<double, Eigen::Matrix<double, 7, 1>>>& gt_data)
        {
            {
                std::string image_path = dataset_path + "associated.txt";
                std::ifstream fin;
                fin.open(image_path);
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

            gt_data.clear();

            std::string gt_path = dataset_path + "pose.txt";
            std::ifstream fin;
            fin.open(gt_path);
            if(!fin.is_open())
            {
                std::cerr << "------------ Reading gt data error --------------" << std::endl;
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

                    double timestamp, tx, ty, tz, qx, qy, qz, qw;
                    ss >> timestamp;
                    ss >> tx; ss >> ty; ss >> tz;
                    ss >> qx; ss >> qy; ss >> qz; ss >> qw;

                    Eigen::Matrix<double, 7, 1> data;
                    data << tx, ty, tz, qx, qy, qz, qw;
                    gt_data.push_back(std::make_pair(timestamp, data));
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


        static void LoadGtCID(const std::string dataset_path, std::vector<Eigen::Matrix<double, 8, 1>>& gt_data)
        {
            gt_data.clear();

            std::string path_gt = dataset_path + "groundtruth.txt";
            std::ifstream fin;
            fin.open(path_gt);
            if(!fin.is_open())
            {
                std::cerr << "------------ Reading gt data error --------------" << std::endl;
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

                    double timestamp, tx, ty, tz, qx, qy, qz, qw;
                    ss >> timestamp;
                    ss >> tx; ss >> ty; ss >> tz;
                    ss >> qx; ss >> qy; ss >> qz; ss >> qw;

                    Eigen::Matrix<double, 8, 1> data;
                    data << timestamp, tx, ty, tz, qx, qy, qz, qw;
                    gt_data.push_back(data);
                }
            }

            fin.close();
        }

        static void LoadImageDataEuroc(const std::string dataset_path, std::vector<std::pair<double, std::string>>& image_data)
        {
            std::string path = dataset_path + "data.csv";
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

        static void LoadGtEuroc(const std::string dataset_path, std::vector<Eigen::Matrix<double, 8, 1>>& gt_data)
        {
            gt_data.clear();

            std::string path_gt = dataset_path + "state_groundtruth_estimate0/data.tum";
            std::ifstream fin;
            fin.open(path_gt);
            if(!fin.is_open())
            {
                std::cerr << "------------ Reading gt data error --------------" << std::endl;
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

                    double timestamp, tx, ty, tz, qx, qy, qz, qw;
                    ss >> timestamp;
                    ss >> tx; ss >> ty; ss >> tz;
                    ss >> qx; ss >> qy; ss >> qz; ss >> qw;

                    Eigen::Matrix<double, 8, 1> data;
                    data << timestamp, tx, ty, tz, qx, qy, qz, qw;
                    gt_data.push_back(data);
                }
            }

            fin.close();
        }


        static void LoadVIOTraj(const std::string file_name, std::vector<Eigen::Matrix<double, 8, 1>>& cam_data)
        {
            cam_data.clear();

            std::ifstream fin;
            fin.open(file_name);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading vio data error--------------" << std::endl;
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

                    double timestamp, tx, ty, tz, qx, qy, qz, qw;
                    ss >> timestamp;
                    ss >> tx; ss >> ty; ss >> tz;
                    ss >> qx; ss >> qy; ss >> qz; ss >> qw;

                    Eigen::Quaterniond q_jpl(qw, qx, qy, qz);  // G->C
//                    Eigen::Matrix<double,3,3> R(q_jpl);
//                    Eigen::Quaterniond q_hamilton(R.transpose());

                    Eigen::Matrix<double, 8, 1> data;
//                    data << timestamp, tx, ty, tz, q_hamilton.x(), q_hamilton.y(), q_hamilton.z(), q_hamilton.w();
                    data << timestamp, tx, ty, tz, q_jpl.x(), q_jpl.y(), q_jpl.z(), q_jpl.w();
                    cam_data.push_back(data);
                }
            }

            fin.close();
        }

        static void LoadVIOTime(const std::string file_name, double& avg_time)
        {

            std::ifstream fin;
            fin.open(file_name);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading vio time error--------------" << std::endl;
                return;
            }

            int cnt = 0;
            double total_time = 0.0;
            std::string str;
            while(!fin.eof()) {
                getline(fin, str);
                if(!str.empty())
                {
                    std::stringstream ss;
                    ss << str;

                    std::string average;
                    double time;

                    ss >> average;
                    ss >> time;

                    total_time += time;
                    cnt++;
                }
            }

            avg_time = total_time / cnt;
            fin.close();
        }

        static void LoadVIOTimeEuroc(const std::string file_name, double& avg_time)
        {

            std::ifstream fin;
            fin.open(file_name);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading vio time error--------------" << std::endl;
                return;
            }

            // 用于存储文件中的所有行
            std::vector<std::string> lines;
            std::string line;

            // 读取文件的每一行
            while (std::getline(fin, line)) {
                lines.push_back(line);
            }

            fin.close();

            // 检查文件是否为空
            if (lines.empty()) {
                std::cerr << "File is empty" << std::endl;
                return;
            }

            // 获取最后一行
            std::string lastLine = lines.back();

            // 使用字符串流来解析最后一行中的数字
            std::stringstream ss(lastLine);
            std::string str1, str2, str3;
            ss >> str1; ss >> str2; ss >> str3;
            ss >> avg_time;
        }

        static void LoadImageDataVCU(const std::string dataset_path, std::vector<std::pair<double, std::string>>& image_data)
        {
            std::string path = dataset_path + "associations.txt";
            std::ifstream fin;
            fin.open(path);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading image data error--------------" << std::endl;
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


        static void LoadImuDataVCU(const std::string dataset_path, std::vector<Eigen::Matrix<double, 7, 1>>& imu_data)
        {
            std::string path_imu = dataset_path + "imu.txt";
            std::ifstream fin;
            fin.open(path_imu);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading IMU data error--------------" << std::endl;
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
//                    data << timestamp, wx, wy, wz, ax * (-9.81), ay * (-9.81), az * (-9.81);
                    data << timestamp, wx, wy, wz, ax, ay, az;
                    imu_data.push_back(data);
                }
            }

            fin.close();
        }


        static void LoadGtVCU(const std::string dataset_path, std::vector<Eigen::Matrix<double, 8, 1>>& gt_data)
        {
            gt_data.clear();

            std::string path_gt = dataset_path + "groundtruth.csv";
            std::ifstream fin;
            fin.open(path_gt);
            if(!fin.is_open())
            {
                std::cerr << "------------Reading gt data error--------------" << std::endl;
                return;
            }

            while(!fin.eof())
            {
                std::string str;
                getline(fin, str);

                if(!str.empty())
                {

                    // Replace all "," for " "
                    size_t pos = str.find(",");
                    while(pos != str.npos)
                    {
                        str.replace(pos, 1, " ");
                        pos = str.find(",", pos+1);
                    }

                    std::stringstream ss;
                    ss << str;

                    double timestamp, tx, ty, tz, qx, qy, qz, qw;
                    ss >> timestamp;
                    ss >> tx; ss >> ty; ss >> tz;
                    ss >> qx; ss >> qy; ss >> qz; ss >> qw;


                    Eigen::Quaterniond q_hamilton(qw, qx, qy, qz);
                    Eigen::Matrix<double,3,3> R(q_hamilton);
                    Eigen::Quaterniond q_jpl(R.transpose());

                    Eigen::Matrix<double, 8, 1> data;
//                    data << timestamp, tx, ty, tz, q_hamilton.x(), q_hamilton.y(), q_hamilton.z(), q_hamilton.w();
                    data << timestamp, tx, ty, tz, q_jpl.x(), q_jpl.y(), q_jpl.z(), q_jpl.w();

                    gt_data.push_back(data);
                }
            }

            fin.close();
        }


    private:

        /**
         * All function in this class should be static.
         * Thus an instance of this class cannot be created.
         */
        DatasetReader() {}

    };

}

#endif //RGBD_PLANE_VIO_DATASET_READER_H
