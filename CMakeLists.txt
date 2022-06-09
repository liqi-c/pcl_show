cmake_minimum_required(VERSION 2.8)
project(pcl_test)

add_definitions(-std=c++11)

# # "option"用来定义宏，"ON"表示打开，"OFF"表示关闭
# option (USE_CROSS_COM "Use cross compile" OFF)
set(USE_CROSS_COM 0) #0表示不使用交叉编译 1表示使用交叉编译

set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

if(${USE_CROSS_COM})
    message("use cross compile!!!!!!!!!!")
    set(CMAKE_C_COMPILER "/usr/local/ARM-toolchain/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "/usr/local/ARM-toolchain/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++")
    # set(OPENNI2_INCLUDE "$ENV{HOME}/AXonOpenNI-Linux-Arm-2.3/Include")
    # set(OPENNI2_REDIST "$ENV{HOME}/AXonOpenNI-Linux-Arm-2.3/Redist")
    # add_definitions(-DWITH_CAMERA_DRIVER) #是否使用相机驱动
endif(${USE_CROSS_COM})

# include_directories(${OPENNI2_INCLUDE})
# link_directories(${OPENNI2_REDIST})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(${USE_CROSS_COM})
    set(PCLPATH "$ENV{HOME}/pcl-arm") #设置pcl库目录,根据自己放pcl库的目录修改
    MESSAGE(STATUS "PCLPATH : ${PCLPATH}")

    set(FLANN_INCLUDE "${PCLPATH}/flann/include")
    set(FLANN_LIB "${PCLPATH}/flann/lib/libflann_cpp_s.a;")
    include_directories(${FLANN_INCLUDE})
    link_directories(${FLANN_LIB})
    #MESSAGE(STATUS "FLANN_LIB : ${FLANN_LIB}")

    set(EIGEN3_INCLUDE "${PCLPATH}/eigen3/include/eigen3")
    include_directories(${EIGEN3_INCLUDE})

    set(PCL_INCLUDE "${PCLPATH}/pcl/include/pcl-1.9")
    set(PCL_LIB "${PCLPATH}/pcl/lib/libpcl_filters.a;${PCLPATH}/pcl/lib/libpcl_search.a;${PCLPATH}/pcl/lib/libpcl_segmentation.a;${PCLPATH}/pcl/lib/libpcl_common.a;${PCLPATH}/pcl/lib/libpcl_kdtree.a;${PCLPATH}/pcl/lib/libpcl_sample_consensus.a;${PCLPATH}/lib/libflann_cpp_s.a;")
    include_directories(${PCL_INCLUDE})
    link_directories(${PCL_LIB})
    #MESSAGE(STATUS "PCL_LIB : ${PCL_LIB}")

    set(BOOST_INCLUDE "${PCLPATH}/include")

    include_directories(${BOOST_INCLUDE})

    #MESSAGE(STATUS "BOOST_LIB : ${BOOST_LIB}")
endif(${USE_CROSS_COM})

include_directories(
    # ${OPENNI2_INCLUDE}
    ./include
    # ./include/camera_driver
    ./include/depth_image_to_pointcloud)

add_library(${PROJECT_NAME}
#    src/main.cpp
    src/isca_tof_camera.cpp
    src/tof_camera.cpp
    # src/camera_driver/camera_driver.cpp
    # src/camera_driver/camera_driver_api.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud_api.cpp
    )
if(${USE_CROSS_COM})
    target_link_libraries(${PROJECT_NAME} ${FLANN_LIB} ${PCL_LIB} ${BOOST_LIB})
endif (${USE_CROSS_COM})

add_executable(test
    src/main.cpp
    src/isca_tof_camera.cpp
    src/tof_camera.cpp
    # src/camera_driver/camera_driver.cpp
    # src/camera_driver/camera_driver_api.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud_api.cpp
    )
if(${USE_CROSS_COM})
    target_link_libraries(test ${CMAKE_CURRENT_SOURCE_DIR}/bin/lib/libaxon.a ${FLANN_LIB} ${PCL_LIB} ${BOOST_LIB})
endif (${USE_CROSS_COM})
#target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${Boost_LIBRARIES} libOpenNI2.so ${PCL_LIBRARIES} )
#add_dependencies(${PROJECT_NAME} OpenNI2)

