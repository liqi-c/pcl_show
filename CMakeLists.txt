cmake_minimum_required(VERSION 2.8)
project(pcl_test)

add_definitions(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    # ${OPENNI2_INCLUDE}
    /usr/local/include/pcl-1.12
    /usr/include/eigen3
    /usr/include/vtk-6.3  ## add for qli enviroment
    ./include
    ./include/depth_image_to_pointcloud)

add_executable(test
    src/main.cpp
    src/pcl_show.cpp 
    src/isca_tof_camera.cpp
    src/tof_camera.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud.cpp
    src/depth_image_to_pointcloud/depth_image_to_pointcloud_api.cpp
)
link_directories(/usr/local/lib)
set(PCL_LIBS "pcl_filters;pcl_search;pcl_segmentation;pcl_common;pcl_kdtree;pcl_sample_consensus;flann_cpp_s;boost_filesystem;pcl_visualization;pcl_io;")
set(VTK_LIBS "vtkCommonCore-6.3;vtkRenderingCore-6.3;vtkCommonDataModel-6.3;vtkCommonMath-6.3")
target_link_libraries(test ${PCL_LIBS} ${VTK_LIBS})