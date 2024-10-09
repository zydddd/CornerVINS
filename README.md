# CornerVINS
**Author:** Yidi Zhang


# 1. Prerequisites
We have tested the library in **Ubuntu 18.04**, but it should be easy to compile in other platforms.
## C++11 or C++0x Compiler
## Boost
We only use boost in Chi2 distance check.
## Eigen3
We use some basic eigen data type. Download and install instructions can be found at: http://eigen.tuxfamily.org.
```
sudo apt-get install libeigen3-dev
```

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features.
Download and install instructions can be found at: http://opencv.org.
**Required at least 3.0. Tested with OpenCV 3.3.1**.
Install dependencies
```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
```
sudo apt-get install libvtk7-dev  # or libvtk6-dev, 7 may conflict with ros environment
```
Download sources at: https://opencv.org/releases/.
```
cd opencv-3.3.1
mkdir build
cd build
cmake -DWITH_VTK=ON ..
make -j
sudo make install
```

## Ceres
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.

Install dependencies
```
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
```
Download at: http://ceres-solver.org. Required **1.14.0**
```
mkdir build
cd build
cmake ..
make -j
sudo make install
```
## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface.
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

Install dependencies
```
sudo apt install libglew-dev
```
Download at: https://github.com/stevenlovegrove/Pangolin. Required **0.6**
```
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

## CornerVINS.so
We have encapsulated some of the code in library CornerVINS.so.
Since the file size is too large, please email the authors to obtain it.

# 2. Building CornerVINS library and examples

Since our VIO system is a CMake project, use the following command to compile the whole system:
```
mkdir build
cd build
cmake .. && make
```
Usages:

Using default <dataset_path> <path_results> <file_traj> in ./config/sims.yaml
```
./bin/test_sims
```
Or
```
./bin/test_sims <path_yaml> <file_yaml>> <dataset_path> <path_results> <file_traj>
```
For example:
```
./bin/test_floor ./config/ floor.yaml /home/zyd/File/data0111/office/office_1/ /home/zyd/File/data0111/office/office_1/results/rvg-vio/test1/ rgbd_test.txt
```
`run_cid.sh` provides a script for testing sequences in CID-SIMS dataset.
The data is structured as:
```
/home/zyd/File/CID_SIMS/office/office_1/
├── color
│   ├── 1673415345.711926.png
│   ├── ...
├── depth
│   ├── 1673415345.711926.png
│   ├── ...
├── associations.txt
├── groundtruth.txt
└── imu.txt
```
