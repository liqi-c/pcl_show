
#ifndef __PCL_SHOW__
#define __PCL_SHOW__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include "isca_tof_camera.h"

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
    float x[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云x坐标序列数据起始指针
    float y[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云y坐标序列数据起始指针
    float z[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云z坐标序列数据起始指针
}s_xyzdata; 

// bool readSunnyXyz(const char * file , s_PointData *read_pcl);
// bool Read_Pcd(const char * file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori );
// bool Convert_SunnyXyz_To_IscasPCL(s_xyzdata * tmp_data ,s_PointData * read_pcl , iscas::PointCloud& pnt_cloud_origin);
// bool Convert_IscasPCL_To_PCL(iscas::PointCloud& iscas_pcl_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_out);
// bool Convert_PCL_To_IscasPCL( pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data , iscas::PointCloud& pointcloud_ori );

// bool PCL_Show_Double_In_1_Windows(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl);
// bool PCL_Show_Double_In_2_Windows(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl);
// bool PCL_Show_Continue_One(pcl::visualization::PCLVisualizer viewer_pvs,pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data);
// bool PCL_Show_Signle(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data);

bool Read_PCD_and_Show_Single(const char * life_path);
bool Read_PCD_and_Show_Double(const char * life_path, const char * right_path);

bool Read_PCD_and_Show_Continue_Single(const char * life_path);
bool Read_PCD_and_Show_Continue_Double(const char * life_path, const char * right_path);
int test_alg(void);

#endif
