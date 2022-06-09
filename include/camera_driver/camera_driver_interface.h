#ifndef CAMERA_DRIVER_INTERFACE_H
#define CAMERA_DRIVER_INTERFACE_H

#include "isca_tof_camera.h"

namespace iscas
{

class CameraDriverInterface
{
public:
    virtual ~CameraDriverInterface() {}

    virtual bool init() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool pause() = 0;
    virtual bool continu() = 0;
    virtual bool getOneFrame(DepthImage& image) = 0;
    virtual bool getCameraParamters(CameraParamter& param) = 0;
protected:
    CameraDriverInterface() {}
};

}

#endif // CAMERA_DRIVER_INTERFACE_H

