#include "depth_image_to_pointcloud_api.h"
#include "depth_image_to_pointcloud/depth_image_to_pointcloud.h"
namespace iscas
{

handle_t depth_image_to_point_cloud_create()
{
    DepthImageToPointCloudInterface* ptr = new DepthImageToPointCloud;
    handle_t h = static_cast<handle_t>(ptr);
    return h;
}

void depth_image_to_point_cloud_destory(handle_t &h)
{
    if (NULL == h)
    {
        return;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    delete ptr;
    ptr = NULL;
    h = NULL;
}

bool depth_image_to_point_cloud_set_paramter(handle_t h, CameraParamter& param)
{
    if (NULL == h)
    {
        return false;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    if (!ptr->setUnitScaling(param.scaling))
    {
        return false;
    }
    if (!ptr->setCF(param.camera_cf))
    {
        return false;
    }
    if (!ptr->setTF(param.camera_tf))
    {
        return false;
    }
    ptr = NULL;
    return true;
}

bool depth_image_to_point_cloud_process_one_frame(handle_t h, DepthImage &image, PointCloud &pointcloud)
{
    if (NULL == h)
    {
        return false;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    PointCloud pointcloud_in;
    bool ret = ptr->transformDepthImageToPointCloud(image, pointcloud_in);
    ret = ptr->processPointCloud(pointcloud_in, pointcloud);
    return ret;
}

bool point_cloud_to_point_cloud_process_one_frame(handle_t h, PointCloud &pointcloud_in, PointCloud &pointcloud)
{
    if (NULL == h)
    {
        return false;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    bool ret = ptr->processPointCloud(pointcloud_in, pointcloud);
    return ret;
}

bool depth_image_to_point_cloud_process_one_frame_origin(handle_t h, DepthImage &image, PointCloud &pointcloud)
{
    if (NULL == h)
    {
        return false;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    bool ret = ptr->transformDepthImageToPointCloud(image, pointcloud);
    return ret;
}

bool depth_image_to_point_cloud_set_paramter2(handle_t h, Param &param)
{
    if (NULL == h)
    {
        return false;
    }
    DepthImageToPointCloudInterface* ptr = static_cast<DepthImageToPointCloudInterface*>(h);
    bool ret = ptr->setParam(param);
    return ret;
}

}



