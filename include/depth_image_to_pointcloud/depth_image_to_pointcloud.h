/**
 * @file depth_image_to_pointcloud.h
 * @author jiangzeyu (jiangzeyu@gziis.org)
 * @brief 点云处理的函数
 * @version 1.0
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef DEPTH_IMAGE_TO_POINTCLOUD_H
#define DEPTH_IMAGE_TO_POINTCLOUD_H

#include "depth_image_to_pointcloud_interface.h"

// #define PCL_VISUALIZOR 1

#include <pcl/ModelCoefficients.h>
#ifdef PCL_VISUALIZOR
#include <pcl/visualization/pcl_visualizer.h>
#endif

namespace iscas
{
class DepthImageToPointCloud : public DepthImageToPointCloudInterface
{
public:
    DepthImageToPointCloud();
    virtual ~DepthImageToPointCloud();

    virtual bool setCF(const CameraCF cf);
    virtual bool setUnitScaling(const double scaling);
    virtual bool setTF(const CameraTF tf);

    /**
     * @brief 将深度图转换为点云
     * 
     * @param image_in 深度图
     * @param pointcloud_out 点云
     * @return true 成功
     * @return false 失败
     */
    virtual bool transformDepthImageToPointCloud(const DepthImage& image_in, PointCloud& pointcloud_out);

    /**
     * @brief 点云处理的函数(调用pcl部分)
     * 
     * @param pointcloud_in 输入的点云
     * @param pointcloud_out 输出的点云
     * @return true 成功处理一帧数据
     * @return false 失败
     */
    virtual bool processPointCloud(const PointCloud &pointcloud_in, PointCloud &pointcloud_out);
    virtual bool setParam(Param param);

    /**
     * @brief Set the Limits object设置截取的点云的范围，高度以地面为0
     * 
     * @param min_x 最小的深度
     * @param max_x 最大的深度
     * @param min_z 最小的高度，最小为-0.2；
     * @param max_z 最大的高度，最大为0.15；
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setLimits(double min_x, double max_x, double min_z, double max_z);

    PointCloud pc_;

private:

    bool is_get_cf_; ///<@brief 是否得到cf
    bool is_get_tf_; ///<@brief 是否得到tf
    bool is_get_scaling_; ///<@brief 是否得到深度分辨率

    CameraCF cf_; ///<@brief 镜头中心点和焦距
    float matrix4f_[4][4]; ///<@brief 存储相机坐标系到深度相机镜头坐标系变换矩阵
    double unit_scaling_; ///<@brief 深度图像素分辨率
    float constant_x_;
    float constant_y_;
    float center_x_; ///<@brief 深度图像素中心点坐标 x
    float center_y_; ///<@brief 深度图像素中心点坐标 y

    unsigned int width_; ///<@brief 深度图的宽
    unsigned int height_;  ///<@brief 深度图的高

    float* x_set; ///< @brief 输出的点云x坐标序列数据起始指针
    float* y_set; ///< @brief 输出的点云y坐标序列数据起始指针
    float* z_set; ///< @brief 输出的点云z坐标序列数据起始指针

    float min_x_; ///<@brief 最小深度，默认0 单位m
    float max_x_; ///<@brief 最大深度，默认0.7 单位m
    float min_z_; ///<@brief 最小高度，默认-0.1 单位m
    float max_z_; ///<@brief 最大高度，默认0.08 单位m

    float size_leaf_;///<@brief 点云下采样参数
    float camera_offset_;///<@brief 相机的距离地面的高度

    pcl::ModelCoefficients model_coefficients_last_;// 上一个时刻的地面的法向量

#ifdef PCL_VISUALIZOR
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    /**
     * @brief 点云显示
     * 
     * @param viewer 显示的句柄
     * @param cloud 需要显示的点云
     * @param color 颜色,目前只设置了"r" "g" "b" "w"
     * @param point_size 点云点显示的大小
     * @param id 点云的id，通过id识别是否同一个点云
     */
    void showCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
					pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string color, double point_size, std::string id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> *single_color_w_, *single_color_r_, *single_color_g_, *single_color_b_;
#endif
};


}

#endif // DEPTH_IMAGE_TO_POINTCLOUD_H

