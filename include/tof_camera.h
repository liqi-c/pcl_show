#ifndef TOF_CAMERA_H
#define TOF_CAMERA_H

#include "tof_camera_interface.h"
namespace iscas
{


class TofCamera : public TofCameraInterface
{
public:

    TofCamera();

    virtual ~TofCamera();
#ifdef WITH_CAMERA_DRIVER
    virtual bool getObstcalePointCloud(
            PointCloud& pnt_cloud, DepthImage& image);
#endif
    virtual bool setRobotPose(PoseStamped& pose);
#ifdef WITH_CAMERA_DRIVER
    virtual bool getOneFrameDepthImage(DepthImage& image);
#endif
    virtual bool cvtDepthImage2PointCloud(DepthImage &image, PointCloud &pnt_cloud);
    virtual bool cvtPointCloud2PointCloud(PointCloud &pnt_cloud_in, PointCloud &pnt_cloud);
    virtual bool cvtDepthImage2PointCloudOrigin(DepthImage &image, PointCloud &pnt_cloud);
    virtual bool setParam(Param param);

private:

};

}

#endif // TOF_CAMERA_H
