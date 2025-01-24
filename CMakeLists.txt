cmake_minimum_required(VERSION 2.8.8)
project(CornerVINS)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(PCL REQUIRED)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message("PCL version  is ： ${PCL_VERSION}")

# Try to compile with c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

# Include our header files
include_directories(
        ${PROJECT_SOURCE_DIR}/Evaluation
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/data_types
        ${PROJECT_SOURCE_DIR}/include/ekf
        ${PROJECT_SOURCE_DIR}/include/point_features
        ${PROJECT_SOURCE_DIR}/include/plane_features
        ${PROJECT_SOURCE_DIR}/include/plane_features/FGM
        ${PROJECT_SOURCE_DIR}/include/initial
        ${PROJECT_SOURCE_DIR}/include/initial/factor
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCULDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


# Set link libraries used by all binaries
list(APPEND ThirdParty_Libraries
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

##################################################
# Make the core library
##################################################
add_subdirectory(app)
##################################################

