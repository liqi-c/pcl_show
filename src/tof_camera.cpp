#include "tof_camera.h"
namespace iscas
{

TofCamera::TofCamera() : TofCameraInterface()
{

}

TofCamera::~TofCamera()
{

}

bool TofCamera::setRobotPose(PoseStamped &pose)
{

}

#ifdef WITH_CAMERA_DRIVER
bool TofCamera::getOneFrameDepthImage(DepthImage &image)
{
    if (!camera_driver_get_one_frame(camera_, image))
    {
        return false;
    }
    return true;
}
#endif

bool TofCamera::cvtDepthImage2PointCloud(DepthImage &image, PointCloud &pnt_cloud)
{
    if (!depth_image_to_point_cloud_process_one_frame(processor_, image, pnt_cloud))
    {
        return false;
    }
    return true;
}

bool TofCamera::cvtPointCloud2PointCloud(PointCloud &pnt_cloud_in, PointCloud &pnt_cloud)
{
    if (!point_cloud_to_point_cloud_process_one_frame(processor_, pnt_cloud_in, pnt_cloud))
    {
        return false;
    }
    return true;
}

bool TofCamera::cvtDepthImage2PointCloudOrigin(DepthImage &image, PointCloud &pnt_cloud)
{
    if (!depth_image_to_point_cloud_process_one_frame_origin(processor_, image, pnt_cloud))
    {
        return false;
    }
    return true;
}

bool TofCamera::setParam(Param param)
{
    if (!depth_image_to_point_cloud_set_paramter2(processor_, param))
    {
        return false;
    }
    return true;
}

#ifdef WITH_CAMERA_DRIVER

bool TofCamera::getObstcalePointCloud(PointCloud &pnt_cloud, DepthImage &image)
{
    if (!camera_driver_get_one_frame(camera_, image))
    {
        return false;
    }
    if (!depth_image_to_point_cloud_process_one_frame(processor_, image, pnt_cloud))
    {
        return false;
    }
    return true;
}
#endif
}
