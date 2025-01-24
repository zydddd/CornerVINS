# CornerVINS

**Authors:** Yidi Zhang, Fulin Tang, Yihong Wu

This is the initial codebase of paper "CornerVINS: Accurate Localization and Layout Mapping for Structural Environments Leveraging Hierarchical Geometric Representations".

The repository construction is in progress. 
We will update the code and normalize this repository as soon as possible. 
Now you can test the project by the following instructions.

| [![Video 1](https://res.cloudinary.com/marcomontalbano/image/upload/v1737707381/video_to_markdown/images/youtube--q5JLuBHbFxs-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/q5JLuBHbFxs) | [![Video 2](https://res.cloudinary.com/marcomontalbano/image/upload/v1737707435/video_to_markdown/images/youtube--nfBJ4QxDRt8-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/nfBJ4QxDRt8) | [![Video 3](https://res.cloudinary.com/marcomontalbano/image/upload/v1737707466/video_to_markdown/images/youtube--NG9TGNDeIps-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/NG9TGNDeIps) |
|---|---|---|

## 1. Prerequisites
We have tested the project in **Ubuntu 18.04** and **Ubuntu 20.04**.

**Boost**: We only use boost in Chi2 distance check.

**Eigen3**: Download and install instructions can be found at: http://eigen.tuxfamily.org.
```
sudo apt-get install libeigen3-dev
```
**OpenCV**:
Download and install instructions can be found at: http://opencv.org.
Required at least 3.0. Tested with OpenCV 3.3.1.

Install dependencies:
```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
```
sudo apt-get install libvtk7-dev
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

**Ceres**:
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.

Install dependencies:
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
**Pangolin**:
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
## 2. Building CornerVINS library and examples
Clone the repository:
```
git clone https://github.com/zydddd/CornerVINS.git
```
Please download the shared object file from  [here](https://drive.google.com/file/d/1DPeP46gTaTSOTJH4XNwSr_cskvurhojT/view?usp=sharing) and put it in the `lib` directory.

Then you can compile the whole system:
```
mkdir build
cd build
cmake .. && make
```

**Usages**:
Using default <dataset_path> <path_results> <file_traj> in ./config/sims.yaml
```
./bin/test_cid
```
Or
```
./bin/test_cid <path_yaml> <file_yaml>> <dataset_path> <path_results> <file_traj> <file_time> 
```
For example:
```
./bin/test_cid ./config/ cid.yaml /home/zyd/File/CID-SIMS/office/office_1/ /home/zyd/Results/CID-SIMS/office/office_1/test1/ test.txt time.txt
```

The data should be structured as:
```
/home/zyd/File/CID_SIMS/office/office_1/
├── color
│   ├── 1673415345.711926.png
│   ├── ...
├── depth
│   ├── 1673415345.711926.png
│   ├── ...
├── associations.txt <color_timestamp [s] depth_timestamp [s]>
├── groundtruth.txt
└── imu.txt
```

## Acknowledgment

This project is built in part on OpenVINS and PEAC AHC.
We thank all the authors for their contribution.
