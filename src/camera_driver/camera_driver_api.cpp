#include "camera_driver_api.h"
#include "camera_driver.h"

namespace iscas
{

handle_t camera_driver_create()
{
    CameraDriverInterface* ptr = new CameraDriver();
    handle_t h = static_cast<handle_t>(ptr);
    return h;
}

void camera_driver_destory(handle_t &h)
{
    if (NULL == h) {
        return;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    delete ptr;
    ptr = NULL;
    h = NULL;
}

bool camera_driver_init(handle_t h)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->init();
    ptr = NULL;
    return ret;
}

bool camera_driver_start(handle_t h)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->start();
    ptr = NULL;
    return ret;
}

bool camera_driver_stop(handle_t h)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->stop();
    ptr = NULL;
    return ret;
}

bool camera_driver_pause(handle_t h)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->pause();
    ptr = NULL;
    return ret;
}

bool camera_driver_continue(handle_t h)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->continu();
    ptr = NULL;
    return ret;}

bool camera_driver_get_one_frame(handle_t h, DepthImage& image)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);

    bool ret = ptr->getOneFrame(image);

    ptr = NULL;
    return ret;
}

bool camera_driver_get_camera_paramters(handle_t h, CameraParamter &param)
{
    if (NULL == h) {
        return false;
    }
    CameraDriverInterface* ptr = static_cast<CameraDriverInterface*>(h);
    bool ret = ptr->getCameraParamters(param);
    ptr = NULL;
    return ret;
}



}

