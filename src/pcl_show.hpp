
#ifndef __PCL_SHOW__
#define __PCL_SHOW__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>

typedef struct{
	float x;
	float y;
	float z;
}s_PointData ;

#define PCL_WIDTH 224
#define PCL_HEIGHT 108
#define PCL_FRAME_LENGTH (PCL_WIDTH*PCL_HEIGHT)
#define MAX_FILENAME_LENGTH 512

typedef struct ori_data
{
    float x[PCL_FRAME_LENGTH];
    float y[PCL_FRAME_LENGTH];
    float z[PCL_FRAME_LENGTH];
}s_xyzdata;

bool Read_PCD_and_Show_Single(const char * life_path);
bool Read_PCD_and_Show_Double(const char * life_path, const char * right_path);

bool Read_PCD_and_Show_Continue_Single(const char * life_path);
bool Read_PCD_and_Show_Continue_Double(const char * life_path, const char * right_path);
bool Read_PCD_and_Show_Signle_Alg(const char * life_path);
bool Read_PCD_and_Show_Continue_Alg(const char * life_path);

#endif
