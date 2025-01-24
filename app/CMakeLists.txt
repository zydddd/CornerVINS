add_executable(test_cid test_cid.cpp)
target_link_libraries(test_cid
#        ${PROJECT_NAME}
        ${ThirdParty_Libraries}
        ${PROJECT_SOURCE_DIR}/lib/libCornerVINS.so
)