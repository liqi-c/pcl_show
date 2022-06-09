#ifndef CAMERA_DRIVER_API_H
#define CAMERA_DRIVER_API_H

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
handle_t camera_driver_create();

/**
 * @brief depth_image_to_point_cloud_destory 析构函数 销毁句柄
 * @param h 句柄 销毁后对象被释放
 */
void camera_driver_destory(handle_t &h);

/**
 * @brief camera_driver_init 初始化相机
 * @param h 句柄 所有接口函数通过句柄调用
 * @return 是否调用成功
 */
bool camera_driver_init(handle_t h);

/**
 * @brief camera_driver_start 使相机开始工作
 * @param h 句柄 所有接口函数通过句柄调用
 * @return 是否调用成功
 */
bool camera_driver_start(handle_t h);

/**
 * @brief camera_driver_stop 使相机停止工作
 * @param h 句柄 所有接口函数通过句柄调用
 * @return 是否调用成功
 */
bool camera_driver_stop(handle_t h);

/**
 * @brief camera_driver_pause 使相机暂停工作
 * @param h 句柄 所有接口函数通过句柄调用
 * @return 是否调用成功
 */
bool camera_driver_pause(handle_t h);

/**
 * @brief camera_driver_continue 使相机继续工作
 * @param h 句柄 所有接口函数通过句柄调用
 * @return 是否调用成功
 */
bool camera_driver_continue(handle_t h);

/**
 * @brief camera_driver_get_one_frame 获取一帧深度图
 * @param h 句柄 所有接口函数通过句柄调用
 * @param image 深度图
 * @return 是否调用成功
 */
bool camera_driver_get_one_frame(handle_t h, DepthImage& image);

/**
 * @brief camera_driver_get_camera_paramters 获取相机参数
 * @param h 句柄 所有接口函数通过句柄调用
 * @param param 相机参数
 * @return 是否调用成功
 */
bool camera_driver_get_camera_paramters(iscas::handle_t h, CameraParamter &param);

}

#ifdef __cplusplus
}
#endif

#endif // CAMERA_DRIVER_API_H
