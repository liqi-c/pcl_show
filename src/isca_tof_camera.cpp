//#include "isca_tof_camera.h"
#include "tof_camera.h"
namespace iscas
{

handle_t tof_camera_create()
{
    TofCameraInterface* ptr = new TofCamera;
    handle_t h = static_cast<handle_t>(ptr);
    return h;
}

bool tof_camera_init(handle_t h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->init();
}

void tof_camera_destroy(handle_t& h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    delete ptr;
    ptr = NULL;
    h = NULL;
}
#ifdef WITH_CAMERA_DRIVER

bool tof_camera_start(handle_t h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->start();
}

bool tof_camera_stop(handle_t h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->stop();
}

bool tof_camera_pause(handle_t h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->pause();
}

bool tof_camera_continue(handle_t h)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->resume();
}

bool tof_camera_get_obstcale_point_cloud(handle_t h, PointCloud &pnt_cloud, DepthImage &image, PoseStamped &pose)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->getObstcalePointCloud(pnt_cloud, image);
}

bool tof_camera_get_one_frame_depth_image(handle_t h, DepthImage &image)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->getOneFrameDepthImage(image);
}
#endif

bool tof_camera_set_paramters(handle_t h, Param param)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->setParam(param);
}

bool tof_camera_cvt_depth_image_to_point_cloud(handle_t h, DepthImage &image, PointCloud &pnt_cloud, PoseStamped &pose)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->cvtDepthImage2PointCloud(image, pnt_cloud);
}

bool tof_camera_cvt_point_cloud_to_point_cloud(handle_t h, PointCloud &pnt_cloud_in, PointCloud &pnt_cloud, PoseStamped &pose)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->cvtPointCloud2PointCloud(pnt_cloud_in, pnt_cloud);
}

bool tof_camera_cvt_depth_image_to_point_cloud_origin(handle_t h, DepthImage &image, PointCloud &pnt_cloud)
{
    if (NULL == h)
    {
        std::cerr << "tof camera API did not construct!" << std::endl;
        return false;
    }
    TofCameraInterface* ptr = static_cast<TofCameraInterface*>(h);
    return ptr->cvtDepthImage2PointCloudOrigin(image, pnt_cloud);
}
}


