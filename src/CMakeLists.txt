set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

add_library(${PROJECT_NAME} SHARED
        # Basic Data Types
        data_types/basic_type.cpp
        data_types/imu.cpp
        data_types/jpl_quat.cpp
        data_types/landmark.cpp
        data_types/pose_jpl.cpp
        data_types/vec.cpp
        data_types/corner.cpp

        #Point Feature && Tracking && Triangulation
        point_features/point_feature.cpp
        point_features/point_feature_database.cpp
        point_features/point_triangulation.cpp
        point_features/point_detector.cpp
        point_features/point_klt_tracker.cpp

        plane_features/plane_detector.cpp
        plane_features/plane_feature.cpp
        plane_features/plane_graph_tracker.cpp
        plane_features/FGM/hungarian.cpp
        plane_features/FGM/util.cpp
        plane_features/FGM/fgm.cpp
        plane_features/utils.cpp
        plane_features/plane_graph.cpp
        plane_features/feature_associator.cpp

        # initial
        initial/feature_manager.cpp
        initial/global_sfm.cpp
        initial/motion_estimator.cpp
        initial/visual_imu_alignment.cpp
        initial/motion_initializer.cpp
        # factor
        initial/factor/pose_local_parameterization.cpp
        initial/factor/projection_factor.cpp


        # ekf
        ekf/propagator.cpp
        ekf/state.cpp
        ekf/state_utils.cpp
        ekf/update_msckf_feature.cpp
        ekf/update_slam_feature.cpp
        ekf/update_plane.cpp
        ekf/update_zupt.cpp
        ekf/update_utility.cpp

        # Utils Classes
        parameters.cpp
        quat_utils.cpp

        # Initializer
        static_initializer.cpp

        # Core
        vio_system.cpp
        visualizer.cpp
        )

target_link_libraries(${PROJECT_NAME} ${ThirdParty_Libraries})