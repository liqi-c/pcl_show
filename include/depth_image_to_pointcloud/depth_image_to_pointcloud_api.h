#ifndef DEPTH_IMAGE_TO_POINTCLOUD_API_H
#define DEPTH_IMAGE_TO_POINTCLOUD_API_H

#include "isca_tof_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace iscas
{
/**
 * @brief depth_image_to_point_cloud_create 构造函数
 * @return 对象的指针 句柄
 */
handle_t depth_image_to_point_cloud_create();

/**
 * @brief depth_image_to_point_cloud_destory 析构函数 销毁句柄
 * @param h 句柄 销毁后对象被释放
 */
void depth_image_to_point_cloud_destory(handle_t &h);

/**
 * @brief depth_image_to_point_cloud_set_paramter
 * @param h 句柄 所有接口函数通过句柄调用
 * @param param 设置参数 参数与所使用相机相关
 * @return 是否调用成功
 */
bool depth_image_to_point_cloud_set_paramter(handle_t h, CameraParamter& param);

/**
 * @brief depth_image_to_point_cloud_process_one_frame 处理一帧深度图 包括将深度图转换成点云数据 并处理点云
 * @param h  句柄 所有接口函数通过句柄调用
 * @param image 深度图 输入量
 * @param pointcloud 点云 输出量
 * @return 是否调用成功
 */
bool depth_image_to_point_cloud_process_one_frame(handle_t h, DepthImage &image, PointCloud &pointcloud);

/**
 * @brief point_cloud_to_point_cloud_process_one_frame 处理一帧点云
 * @param h  句柄 所有接口函数通过句柄调用
 * @param pointcloud_in 点云 输入量
 * @param pointcloud 点云 输出量
 * @return 是否调用成功
 */
bool point_cloud_to_point_cloud_process_one_frame(handle_t h, PointCloud &pointcloud_in, PointCloud &pointcloud);

/**
 * @brief depth_image_to_point_cloud_process_one_frame_origin 处理一帧深度图 包括将深度图转换成点云数据 不处理点云
 * @param h  句柄 所有接口函数通过句柄调用
 * @param image 深度图 输入量
 * @param pointcloud 点云 输出量
 * @return 是否调用成功
 */
bool depth_image_to_point_cloud_process_one_frame_origin(handle_t h, DepthImage &image, PointCloud &pointcloud);

/**
 * @brief depth_image_to_point_cloud_set_paramter2 用于设置点云处理的参数
 * @param h 句柄 所有接口函数通过句柄调用
 * @param param 需要设置的参数
 * @return 是否调用成功 
 */
bool depth_image_to_point_cloud_set_paramter2(handle_t h, Param &param);

}



#ifdef __cplusplus
}
#endif

#endif // DEPTH_IMAGE_TO_POINTCLOUD_API_H

