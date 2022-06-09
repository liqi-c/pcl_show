#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include "camera_driver_interface.h"

#include <OpenNI.h>
#include "AXonLink.h"
//#include <iostream>
//#include <vector>
//#include <time.h>

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

namespace iscas
{

class CameraDriver : public CameraDriverInterface
{
public:
    CameraDriver();
    CameraDriver(std::string device_URI);
    virtual ~CameraDriver();

    virtual bool init();
    virtual bool start();
    virtual bool stop();
    virtual bool pause();
    virtual bool continu();
    virtual bool getOneFrame(DepthImage& image);
    bool getOneFrame(DepthImage &image, const unsigned char* data_set);
    virtual bool getCF(CameraCF& camera_cf);
    virtual bool setDepthImageFrameID(std::string frame_id);
    virtual bool getCameraParamters(CameraParamter& param);
    void getCamera2DepthOpticalTF(CameraTF& tf);

    bool isInitialized();
    bool getDepthFrameRef(openni::VideoFrameRef& vff);
    bool getAXonLinkCamParam(AXonLinkCamParam& axon_param);
    bool getDepthResolution(double& resolution);

protected:

    bool getPixelResolution(double& pixel_resolution);

    bool is_initializ_;
    bool is_start_;

    CameraTF camToptical_;

    double pixel_resolution_; ///<@brief 深度图像素分辨率

    std::string device_URI_; ///< @brief 设备URI

    openni::Device device_; ///< @brief 设备
    openni::VideoStream depth_stream_; ///< @brief 深度数据流

    OniVersion drver_version_; ///< @brief 驱动版本号
    const openni::SensorInfo* sensor_info_; ///< @brief 传感器信息
    AXonLinkCamParam cam_params_;  ///<@brief 相机内参
    openni::VideoFrameRef m_frame_; ///<@brief 存储最新的深度数据
    std::string depth_image_frame_id_; ///<@brief 深度图所在的坐标系id
};


}

#endif // CAMERA_DRIVER_H

