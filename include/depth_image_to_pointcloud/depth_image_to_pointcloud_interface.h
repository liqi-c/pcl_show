#ifndef DEPTH_IMAGE_TO_POINTCLOUD_INTERFACE_H
#define DEPTH_IMAGE_TO_POINTCLOUD_INTERFACE_H

#include "isca_tof_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace iscas
{

class DepthImageToPointCloudInterface
{
public:
  virtual ~DepthImageToPointCloudInterface() {}
  virtual bool setCF(const CameraCF cf) = 0;
  virtual bool setUnitScaling(const double scaling) = 0;
  virtual bool setTF(const CameraTF tf) = 0;
  virtual bool transformDepthImageToPointCloud(const DepthImage& image_in, PointCloud& pointcloud_out) = 0;
  virtual bool processPointCloud(const PointCloud &pointcloud_in, PointCloud &pointcloud_out) = 0;
  virtual bool setParam(Param param) = 0;

protected:
  DepthImageToPointCloudInterface() {}
};

}

#ifdef __cplusplus
}
#endif

#endif // DEPTH_IMAGE_TO_POINTCLOUD_INTERFACE_H

