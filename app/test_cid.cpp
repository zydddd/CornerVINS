//
// Created by zyd on 23-8-10.
//

#include "dataset_reader.h"
#include "visualizer.h"
#include "parameters.h"
#include "vio_system.h"

/**
 * Example: ./run_cid [config path] [config name] [dataset path] [save path] [traj save name] [time save name]
 * argv[1]: config path
 * argv[2]: config name
 * argv[3]: dataset path
 * argv[4]: save path
 * argv[5]: traj save name
 * argv[6]: time save name
*/

bool upspeed = true;

using namespace Rvg;

int main(int argc, char** argv)
{

    /// 1. Load configurations
    std::string configPath = "../config/";
    std::string configName = "cid.yaml";
    std::string datasetPath, resultsPath, outTrajFile, outTimeFile;
    bool autoExitFlag = false;

    if(argc == 7)
    {
        std::cout << "Read path from terminal." << std::endl;
        configPath = std::string(argv[1]);
        configName = std::string(argv[2]);
        datasetPath = std::string(argv[3]);
        resultsPath = std::string(argv[4]);
        outTrajFile = resultsPath + std::string(argv[5]);
        outTimeFile = resultsPath + std::string(argv[6]);
        autoExitFlag = true;
    }

    cv::FileStorage configParams(configPath + configName, cv::FileStorage::READ);

    if(argc != 7)
    {
        std::cout << "Read path from yaml file." << std::endl;
        datasetPath = std::string(configParams["path_dataset"]);
        resultsPath = std::string(configParams["path_results"]);
        outTrajFile = resultsPath + std::string(configParams["file_traj"]);
        outTimeFile = resultsPath + std::string(configParams["file_time"]);
    }

    std::cout << "datasetPath: " << datasetPath << std::endl;


    bool saveTraj = false;
    configParams["save_traj"] >> saveTraj;
    std::ofstream out_txt_file, out_time_file;
    if(saveTraj)
    {
        std::cout << "outTrajFile: " << outTrajFile << std::endl;
        out_txt_file.open(outTrajFile, std::ios::out | std::ios::trunc);
        out_txt_file << std::fixed;
        out_time_file.open(outTimeFile, std::ios::out | std::ios::trunc);
        out_time_file << std::fixed;
    }


    /// 2. Load horizon floor data
    std::cout << "============= Loading IMU and image data ============" << std::endl;
    std::vector<std::pair<double, std::string> > imageData;  // timestamp, img_name
    std::vector<Eigen::Matrix<double, 7, 1> > imuData;
    CIDDatasetReader::LoadImageDataCID(datasetPath, imageData);
    CIDDatasetReader::LoadImuDataCID(datasetPath, imuData);
    std::cout << "Image size = " << imageData.size() << std::endl;
    std::cout << "IMU size = " << imuData.size() << std::endl;


    /// 3. VIO System and Visualizer
    Parameters params = Parameters::ReadParametersFromYaml(configPath, configName, resultsPath);
    std::shared_ptr<Rvg::VIOSystem> sys = std::make_shared<Rvg::VIOSystem>(params);
    bool has_gt = false;
    configParams["has_gt"] >> has_gt;
    bool hasDepth = params.has_depth;
    bool isVisualize = false;
    configParams["is_visualize"] >> isVisualize;
    Visualizer *viz = new Visualizer();
    if(isVisualize)
        viz = new Visualizer(sys, configPath + configName, autoExitFlag);


    /// 4. Buffer variables for our system (so we always have imu data to use)
    bool hasImage = false;
    cv::Mat imgColor;
    cv::Mat imgColorBuffer;
    cv::Mat imgDepth;
    cv::Mat imgDepthBuffer;
    double time;
    double timeBuffer;

    /// 5. set index of start process image and end index
    int indexImu = 0;
    int indexImg = configParams["img_start"];
    int indexEnd = configParams["img_end"];
    if(indexEnd == 0)
    {
        indexEnd = int(imageData.size());
    }
    std::cout << "Start at: " << indexImg << std::endl;

    double time_cnt = 0.0;
    int cnt = 0;
    /// 6. Main Process
    // step by step to process image and imu data
    std::vector<Eigen::Matrix<double, 8, 1>> camTrajData;
    while (indexImu < (int)imuData.size() && indexImg < indexEnd)  //
    {
        if (imuData[indexImu](0) < imageData[indexImg].first)  //
        {
            // 6.1 send imu data
            double time = imuData[indexImu](0);
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << imuData[indexImu](1), imuData[indexImu](2), imuData[indexImu](3);
            am << imuData[indexImu](4), imuData[indexImu](5), imuData[indexImu](6);

            sys->ProcessImuMeasurement(time, wm, am);
            indexImu++;
        }
        else
        {
            // 6.2 send image data
            std::string colorPath = datasetPath + "color/" + imageData[indexImg].second + ".png";
            imgColor = cv::imread(colorPath, cv::IMREAD_COLOR);
            if (imgColor.empty())
            {
                std::cerr << "------ Failed to load color image ------" << std::endl;
                indexImg++;
                continue;
            }


            if(hasDepth)
            {
                std::string depthPath = datasetPath + "depth/" + imageData[indexImg].second + ".png";
                imgDepth = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
                if (imgDepth.empty())
                {
                    std::cerr << "------ Failed to load depth image ------" << std::endl;
                    indexImg++;
                    continue;
                }
            }

            hasImage = true;
            time = imageData[indexImg].first;
            indexImg++;

        }

        // 6.3 Fill our buffer if we have not (first image)
        if (hasImage && imgColorBuffer.rows == 0)
        {
            hasImage = false;
            timeBuffer = time;
            imgColorBuffer = imgColor.clone();
            if(hasDepth)
            {
                imgDepthBuffer = imgDepth.clone();
            }
        }


        // If we are in monocular mode, then we should process the left if we have it
        if (hasImage)
        {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            if(hasDepth) // RGBD mode
            {
                sys->ProcessCameraMeasurement(timeBuffer, imgColorBuffer, imgDepthBuffer);
            }
            else  // RGB mode
            {
                sys->ProcessCameraMeasurement(timeBuffer, imgColorBuffer);
            }
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


            if (isVisualize) viz->Visualize();

            // save data to txt file
            if(sys->Initialized() && saveTraj)
            {
                double duration = std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count() * 1000;
                out_time_file << std::setprecision(6) << timeBuffer << " " << std::setprecision(3) << duration << std::endl;
                time_cnt += duration;
                cnt ++;

                // IMU Framework Pose
                Eigen::Matrix<double,3,3> R_GtoI = sys->GetState()->imu_->GetRotation();
                Eigen::Matrix<double,3,1> p_IinG = sys->GetState()->imu_->GetPos();

                // save camera pose in global framework to txt
                Eigen::Matrix<double,3,3> R_GtoCi = sys->GetState()->calibImuToCam_->GetRotation() * sys->GetState()->imu_->GetRotation();
                Eigen::Matrix<double,3,1> p_CioinG = sys->GetState()->imu_->GetPos() - R_GtoCi.transpose() * sys->GetState()->calibImuToCam_->GetPos();
                Eigen::Vector4d quat_Rot = QuatUtils::Rot2Quat(R_GtoCi);

                // timestamp(s) tx ty tz qx qy qz qw
                out_txt_file << std::setprecision(6) << sys->GetState()->timestamp_ << " ";
                out_txt_file << std::setprecision(15) << p_CioinG.x() << " " << p_CioinG.y() << " " << p_CioinG.z() << " "
                             << quat_Rot(0) << " " << quat_Rot(1) << " " << quat_Rot(2) << " " << quat_Rot(3)
                             << std::endl;

                if(has_gt)
                {
                    Eigen::Matrix<double, 8, 1> data;
                    data << sys->GetState()->timestamp_, p_CioinG.x(), p_CioinG.y(), p_CioinG.z(),
                    quat_Rot(0), quat_Rot(1), quat_Rot(2), quat_Rot(3);
                    camTrajData.push_back(data);
                }


            }

            // move buffer forward
            hasImage = false;
            timeBuffer = time;
            imgColorBuffer = imgColor.clone();
            if(hasDepth)
            {
                imgDepthBuffer = imgDepth.clone();
            }

        }  //hasImage
    } //while

    if(saveTraj)
    {
        std::cout << std::endl << std::endl << "The trajectory has saved at " << outTrajFile << std::endl;
        out_txt_file.close();

//        out_time_file << "\nAverage: " << std::setprecision(3) << time_cnt / cnt << std::endl;
        out_time_file.close();

    }

    if(isVisualize)
        viz->VisualizeFinal();
    delete viz;

    return 0;
}