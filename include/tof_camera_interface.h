#ifndef TOF_CAMERA_INTERFACE_H
#define TOF_CAMERA_INTERFACE_H
#include "depth_image_to_pointcloud/depth_image_to_pointcloud_api.h"
#include "camera_driver/camera_driver_api.h"

namespace iscas
{

class TofCameraInterface
{
public:

    TofCameraInterface()
    {
#ifdef WITH_CAMERA_DRIVER
        camera_ = camera_driver_create();
#endif
        processor_ = depth_image_to_point_cloud_create();
    }

    virtual ~TofCameraInterface()
    {
#ifdef WITH_CAMERA_DRIVER
        camera_driver_destory(camera_);
#endif
        depth_image_to_point_cloud_destory(processor_);
    }

    virtual bool init()
    {
        CameraParamter param;
#ifdef WITH_CAMERA_DRIVER
        if (!camera_driver_init(camera_))
        {
            return false;
        }
        if (!camera_driver_get_camera_paramters(camera_, param))
        {
            return false;
        }
#endif
        if (!depth_image_to_point_cloud_set_paramter(processor_, param))
        {
            return false;
        }
        return true;
    }

#ifdef WITH_CAMERA_DRIVER
    virtual bool start()
    {
        return  camera_driver_start(camera_);
    }

    virtual bool stop()
    {
        return camera_driver_stop(camera_);
    }

    virtual bool pause()
    {
        return camera_driver_pause(camera_);
    }

    virtual bool resume()
    {
        return camera_driver_continue(camera_);
    }

    virtual bool getObstcalePointCloud(
            PointCloud& pnt_cloud, DepthImage& image) = 0;
#endif

    virtual bool setRobotPose(PoseStamped& pose) = 0;

    virtual bool setCameraTf(CameraTF tf)
    {
        tf_ = tf;
    }
#ifdef WITH_CAMERA_DRIVER
    virtual bool getOneFrameDepthImage(DepthImage& image) = 0;
#endif
    virtual bool cvtDepthImage2PointCloud(DepthImage &image, PointCloud &pnt_cloud) = 0;

    virtual bool cvtPointCloud2PointCloud(PointCloud &pnt_cloud_in, PointCloud &pnt_cloud) = 0;

    virtual bool cvtDepthImage2PointCloudOrigin(DepthImage &image, PointCloud &pnt_cloud) = 0;

    virtual bool setParam(Param param) = 0;
protected:

    handle_t camera_;

    handle_t processor_;

    CameraTF tf_;

};

}
#endif // TOF_CAMERA_INTERFACE_H
