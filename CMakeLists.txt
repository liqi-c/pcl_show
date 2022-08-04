cmake_minimum_required(VERSION 2.8)
project(pcl_show)

add_definitions(-std=c++14)

set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

set(CMAKE_CXX_FLAGS_DEBUG "-g")

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    /usr/local/include/pcl-1.12
    /usr/include/eigen3
    /usr/include/vtk-6.3
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/pcl_show.cpp
    src/pcl_remove_groud.cpp
)
link_directories(/usr/local/lib)
set(PCL_LIBS "pcl_filters;pcl_search;pcl_segmentation;pcl_common;pcl_kdtree;pcl_sample_consensus;flann_cpp_s;boost_filesystem;pcl_visualization;pcl_io;")
set(VTK_LIBS "vtkCommonCore-6.3;vtkRenderingCore-6.3;vtkCommonDataModel-6.3;vtkCommonMath-6.3")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBS} ${VTK_LIBS})