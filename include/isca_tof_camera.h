#ifndef ISCAS_TOF_CAMERA_H
#define ISCAS_TOF_CAMERA_H

#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

namespace iscas {

typedef void* handle_t;

typedef long long int uint64_t;

/**
 * @brief 点云数据格式
 */
typedef struct _PointCloud
{
    float* x_set; ///< @brief 点云x坐标序列数据起始指针
    float* y_set; ///< @brief 点云y坐标序列数据起始指针
    float* z_set; ///< @brief 点云z坐标序列数据起始指针
    int size; ///< @brief 点云数据的长度 即点的个数
    int width; ///< @brief 宽
    int height; ///< @brief 高
    uint64_t time_stamp; ///< @brief 时间戳
} PointCloud;

/**
 * @brief 深度图数据格式
 */
typedef struct _DepthImage
{
    const unsigned char* data_set; ///< @brief 数据起始指针
    int size;  ///< @brief 数据的长度
    int width; ///< @brief 深度图宽
    int height; ///< @brief 深度图高
    double pixel_resolution; ///< @brief 像素分辨率 单位 毫米
    uint64_t time_stamp; ///< @brief 时间戳
} DepthImage;

/**
 * @brief 位姿数据格式
 */
typedef struct _PoseStamped
{
    float x; ///< @brief x坐标
    float y; ///< @brief y坐标
    float z; ///< @brief z坐标
    float yaw; ///< @brief 欧拉角中的偏航角
    float pitch; ///< @brief 欧拉角中的俯仰角
    float roll; ///< @brief 欧拉角中的翻滚角
    uint64_t time_stamp; ///< @brief 时间戳
} PoseStamped;

/**
 * @brief 相机内参
 */
typedef struct _CameraCF
{
    float fx; ///< @brief 相机内参fx
    float fy; ///< @brief 相机内参fy
    float cx; ///< @brief 相机内参cx
    float cy; ///< @brief 相机内参cy
} CameraCF;

/**
 * @brief 相机外参
 */
typedef struct _CameraTF
{
    float x; ///< @brief x坐标
    float y; ///< @brief y坐标
    float z; ///< @brief z坐标
    float yaw; ///< @brief 欧拉角中的偏航角
    float pitch; ///< @brief 欧拉角中的俯仰角
    float roll; ///< @brief 欧拉角中的翻滚角
} CameraTF;

/**
 * @brief 相机参数
 */
typedef struct _CameraParamter
{
    CameraCF camera_cf; ///< @brief CameraCF
    CameraTF camera_tf; ///< @brief CameraTF
    double scaling; ///< @brief 像素分辨率
} CameraParamter;

/**
 * @brief 设置点云处理的参数
 */
typedef struct _Param
{
    float min_hight; ///< @brief 点云的最小高度
    float max_hight; ///< @brief 点云的最大高度
    float min_depth; ///< @brief 点云的最小深度
    float max_depth; ///< @brief 点云的最大深度
    CameraParamter camera_param; ///< @brief 相机参数
    float size_leaf;///<@brief 点云下采样参数
    float camera_offset;///<@brief 相机的距离地面的高度
} Param;

/**
 * @brief tof_camera_create 构造函数 返回一个句柄
 * @return 句柄 接口中其他函数的调用都需要使用此句柄调用
 */
handle_t tof_camera_create();

/**
 * @brief tof_camera_init 初始化相机
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @return 是否调用成功
 */
bool tof_camera_init(handle_t h);

/**
 * @brief tof_camera_destroy 析构函数 销毁句柄
 * @param h 句柄 销毁后指向的对象被释放
 */
void tof_camera_destroy(handle_t& h);

/**
 * @brief tof_camera_start 使相机开始工作
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @return 是否调用成功
 */
bool tof_camera_start(handle_t h);

/**
 * @brief tof_camera_stop 使相机停止工作
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @return 是否调用成功
 */
bool tof_camera_stop(handle_t h);

/**
 * @brief tof_camera_pause 使相机暂停工作
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @return 是否调用成功
 */
bool tof_camera_pause(handle_t h);

/**
 * @brief tof_camera_continue 使相机继续工作
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @return 是否调用成功
 */
bool tof_camera_continue(handle_t h);

/**
 * @brief tof_camera_get_obstcale_point_cloud 获取一帧数据 包括深度图和处理过的点云数据
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param pnt_cloud 点云
 * @param image 深度图
 * @param pose 机器人位姿
 * @return 是否调用成功
 */
bool tof_camera_get_obstcale_point_cloud(
        handle_t h,PointCloud& pnt_cloud,
        DepthImage& image,
        PoseStamped &pose);

/**
 * @brief tof_camera_get_obstcale_point_cloud 获取一帧深度图
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param image 深度图
 * @return 是否调用成功
 */
bool tof_camera_get_one_frame_depth_image(
        handle_t h, DepthImage& image);

/**
 * @brief tof_camera_get_obstcale_point_cloud 深度图处理点云数据
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param image 深度图
 * @param pnt_cloud 点云
 * @param pose 机器人位姿
 * @return 是否调用成功
 */
bool tof_camera_cvt_depth_image_to_point_cloud(
        handle_t h, DepthImage& image,
        PointCloud& pnt_cloud,
        PoseStamped &pose);

/**
 * @brief tof_camera_cvt_point_cloud_to_point_cloud 直接处理点云数据
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param pnt_cloud_in 输入的原始深度图
 * @param pnt_cloud 点云
 * @param pose 机器人位姿
 * @return 是否调用成功
 */
bool tof_camera_cvt_point_cloud_to_point_cloud(
        handle_t h, PointCloud &pnt_cloud_in,
        PointCloud& pnt_cloud,
        PoseStamped &pose);

/**
 * @brief tof_camera_set_paramters 设置参数
 * 
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param param 需要设置的参数
 * @return 是否调用成功 
 */
bool tof_camera_set_paramters(handle_t h, Param param);

/**
 * @brief tof_camera_get_obstcale_point_cloud 深度图转换点云数据（不做处理）
 * @param h 句柄 接口函数的调用都需要使用此句柄调用
 * @param image 深度图
 * @param pnt_cloud 原始的点云
 * @return 是否调用成功
 */
bool tof_camera_cvt_depth_image_to_point_cloud_origin(
        handle_t h, DepthImage& image,
        PointCloud& pnt_cloud);


}




#ifdef __cplusplus
}
#endif


#endif // ISCAS_TOF_CAMERA_H

