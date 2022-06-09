#include "camera_driver.h"
#include <iostream>
#include <cmath>

namespace iscas
{

CameraDriver::CameraDriver():
    is_initializ_(false),
    is_start_(false),
    pixel_resolution_(0),
    device_URI_(""),
    sensor_info_(nullptr)
{
    init();
}

CameraDriver::CameraDriver(std::string device_URI):
    is_initializ_(false),
    is_start_(false),
    pixel_resolution_(0),
    device_URI_(device_URI),
    sensor_info_(nullptr)
{
    init();
}

CameraDriver::~CameraDriver()
{
    depth_stream_.stop();/*停止深度流*/
    depth_stream_.destroy(); /*销毁深度流*/
    device_.close(); /*关闭设备*/
    openni::OpenNI::shutdown(); /*关闭 openni*/
}

bool iscas::CameraDriver::init()
{
    if(is_initializ_)
    {
        std::printf("iscas::CameraDriver::camera_driver_init: The camera has been initialized.\r\n");
        return true;
    }

    //设置默认镜头到相机坐标系变换关系
    camToptical_.x = 0;
    camToptical_.y = -0.0055f;
    camToptical_.z = 0;
    camToptical_.yaw = -1.570796326794896619231321691639751442f;
    camToptical_.pitch = 0;
    camToptical_.roll = -1.570796326794896619231321691639751442f;

    depth_image_frame_id_ = std::string("depth_optical");

    openni::Status rc = openni::STATUS_OK;

    rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        std::printf("iscas::CameraDriver::camera_driver_init: Initialize failed: %s \r\n", openni::OpenNI::getExtendedError());
        return false;
    }

    rc = openni::STATUS_NO_DEVICE;
    while (1)
    {
        if (device_URI_.length() > 0)
        {
            rc = device_.open(device_URI_.c_str());
        }
        else
        {
            rc = device_.open(openni::ANY_DEVICE);
        }

        if(rc == openni::STATUS_OK)
        {
            break;
        }

        printf("iscas::CameraDriver::camera_driver_init: Device open failed: %s\n", openni::OpenNI::getExtendedError());
        sleep(5);
    }

    //获取驱动版本号
    int nsize;
    nsize = sizeof(drver_version_);
    device_.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION, &drver_version_, &nsize);
    std::printf("iscas::CameraDriver::camera_driver_init: camera driver version V%d.%d.%d.%d\n",
                drver_version_.major, drver_version_.minor, drver_version_.maintenance, drver_version_.build);

    //获取传感器信息
    sensor_info_ = device_.getSensorInfo(openni::SENSOR_DEPTH);
    if (sensor_info_ == nullptr)
    {
        std::printf("iscas::CameraDriver::camera_driver_init: getSensorInfo failed.\r\n");
        openni::OpenNI::shutdown();
        return false;
    }
    std::printf("iscas::CameraDriver::camera_driver_init: sensor type: %d\r\n",sensor_info_->getSensorType());

    //获取相机内参
    int dataSize = sizeof(AXonLinkCamParam);
    rc=device_.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,&cam_params_,&dataSize);
    if (rc != openni::STATUS_OK)
    {
        printf("iscas::CameraDriver::camera_driver_init: Couldn't getProperty: %s\n",
               openni::OpenNI::getExtendedError());

        depth_stream_.destroy(); /*关闭深度数据流*/
        device_.close(); /*关闭设备*/
        openni::OpenNI::shutdown(); /*关闭 openni*/
        return false;
    }

    //创建深度流
    rc = depth_stream_.create(device_, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK)
    {
        std::printf("iscas::CameraDriver::camera_driver_init: Couldn't create depth stream: %s\r\n",
                    openni::OpenNI::getExtendedError());

        device_.close(); /*关闭设备*/
        openni::OpenNI::shutdown(); /*关闭 openni*/
        return false;
    }

    std::printf("iscas::CameraDriver::camera_driver_init: The camera initialize succeed.\r\n");
    is_initializ_ = true;

    return true;
}

bool CameraDriver::start()
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::camera_driver_start: no initializ camera.");
        return false;
    }

//    AXonCropping depthAC ;
//    depthAC.originX = 0;
//    depthAC.originY = 0;
//    depthAC.width = 640;
//    depthAC.height = 480;
//    depthAC.gx = 0;
//    depthAC.gy = 0;
//    depth_stream_.setProperty(AXONLINK_STREAM_PROPERTY_CROPPING,depthAC);

    // 设置深度图像视频模式
    openni::VideoMode mModeDepth;
    // 分辨率大小
    mModeDepth.setResolution(640, 480);
    // 每秒30帧
    mModeDepth.setFps(30);
    // 像素格式
    mModeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_3_MM);

    depth_stream_.setMirroringEnabled(true);
    depth_stream_.setVideoMode(mModeDepth);

    openni::Status rc = openni::STATUS_OK;
    rc = depth_stream_.start();
    if (rc != openni::STATUS_OK)
    {
        std::printf("iscas::CameraDriver::camera_driver_start: Couldn't start depth stream: %s\n",
                    openni::OpenNI::getExtendedError());

        return false;
    }
    is_start_ = true;
    std::printf("iscas::CameraDriver::camera_driver_start: depth stream start.\n");

    return true;
}

bool CameraDriver::stop()
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::camera_driver_stop: no initializ camera.");
        return false;
    }
    if(!is_start_)
        return true;
    depth_stream_.stop();/*停止深度流*/
    is_start_ = false;
    std::printf("iscas::CameraDriver::camera_driver_stop: depth stream stop.\n");
    return true;
}

bool CameraDriver::pause()
{
    return stop();
}

bool CameraDriver::continu()
{
    return start();
}

bool CameraDriver::getOneFrame(DepthImage &image)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::camera_driver_get_one_frame: no initializ camera.");
        return false;
    }

    if(!getDepthFrameRef(m_frame_))
    {
        std::printf("iscas::CameraDriver::camera_driver_get_one_frame: getDepthFrameRef failed.\n");
        return false;
    }

    openni::PixelFormat pixel_for_mat = m_frame_.getVideoMode().getPixelFormat();

    double pixel_resolution = 0;

    if(pixel_for_mat == openni::PIXEL_FORMAT_DEPTH_1_3_MM)
    {
        pixel_resolution = 1.0/3.0;
    }
    else if(pixel_for_mat == openni::PIXEL_FORMAT_DEPTH_1_MM)
    {
        pixel_resolution = 1.0;
    }
    else if(pixel_for_mat != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        pixel_resolution = 0.1;
    }
    else
    {
        std::printf("iscas::CameraDriver::camera_driver_get_one_frame: Unexpected frame format\n");
        return false;
    }

    image.width = m_frame_.getWidth();
    image.height = m_frame_.getHeight();
    image.pixel_resolution = pixel_resolution;
    image.data_set = reinterpret_cast<const unsigned char*>(m_frame_.getData());
    image.size = m_frame_.getWidth()*m_frame_.getHeight();
    image.time_stamp = static_cast<long long int>(m_frame_.getTimestamp());

    return true;
}

bool CameraDriver::getOneFrame(DepthImage &image, const unsigned char* data_set)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::camera_driver_get_one_frame: no initializ camera.");
        return false;
    }

    image.width = 640;
    image.height = 480;
    image.pixel_resolution = 1.0/3.0;
    image.data_set = reinterpret_cast<const unsigned char*>(data_set);
    image.size = image.width * image.height;
    image.time_stamp = 0;

    return true;
}

bool CameraDriver::getCF(CameraCF& camera_cf)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::getAXonLinkCamParam: no initializ camera.");
        return false;
    }

    camera_cf.fx = cam_params_.astDepthParam->fx;
    camera_cf.fy = cam_params_.astDepthParam->fy;
    camera_cf.cx = cam_params_.astDepthParam->cx;
    camera_cf.cy = cam_params_.astDepthParam->cy;
    return true;
}

bool CameraDriver::setDepthImageFrameID(std::string frame_id)
{
    depth_image_frame_id_ = frame_id;
    return true;
}

bool CameraDriver::getCameraParamters(CameraParamter &param)
{
    getDepthResolution(param.scaling);
    getCF(param.camera_cf);
    getCamera2DepthOpticalTF(param.camera_tf);
    //  std::cout << "scaling: " << param.scaling << std::endl;
    //  std::cout << "camera cf: " << param.camera_cf.cx << std::endl;
    //  std::cout << "camera tf: " << param.camera_tf.y << std::endl;
    //  strcpy(param.frame_id, depth_image_frame_id_.c_str());
    return true;
}

void CameraDriver::getCamera2DepthOpticalTF(CameraTF& tf)
{
    tf = camToptical_;
}

bool CameraDriver::isInitialized()
{
    return is_initializ_;
}

bool CameraDriver::getDepthFrameRef(openni::VideoFrameRef &vff)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::getDepthFrameRef: no initializ camera.");
        return false;
    }
    openni::Status rc = openni::STATUS_OK;

    int changedStreamDummy;
    openni::VideoStream* pStream = &depth_stream_;
    rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT); //等待流
    if (rc != openni::STATUS_OK)
    {
        std::printf("iscas::CameraDriver::getDepthFrameRef:Wait failed! (timeout is %d ms) %s\n",
                    SAMPLE_READ_WAIT_TIMEOUT, openni::OpenNI::getExtendedError());

        return false;
    }

    rc = depth_stream_.readFrame(&vff); /*读深度流帧数据*/
    if (rc != openni::STATUS_OK)
    {
        printf("iscas::CameraDriver::getDepthFrameRef:readFrame failed! %s\n",
               openni::OpenNI::getExtendedError());

        return false;
    }
    return true;
}

bool CameraDriver::getAXonLinkCamParam(AXonLinkCamParam &axon_param)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::getAXonLinkCamParam: no initializ camera.");
        return false;
    }
    axon_param = cam_params_;
    return true;
}

bool CameraDriver::getDepthResolution(double &resolution)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::getDepthResolution: no initializ camera.");
        return false;
    }
    if(pixel_resolution_ > 0)
    {
        resolution = pixel_resolution_;
        return true;
    }
    return getPixelResolution(resolution);
}

bool CameraDriver::getPixelResolution(double& pixel_resolution)
{
    if(!is_initializ_)
    {
        std::printf("iscas::CameraDriver::getPixelResolution: no initializ camera.");
        return false;
    }
    bool current_is_start = is_start_;
    if(!current_is_start)
    {
        if(!start())
        {
            std::printf("iscas::CameraDriver::getPixelResolution: camera_driver_start error.\r\n");
            return false;
        }
    }

    openni::Status rc = openni::STATUS_OK;

    int changedStreamDummy;
    openni::VideoStream* pStream = &depth_stream_;
    rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT); //等待流
    if (rc != openni::STATUS_OK)
    {
        std::printf("iscas::CameraDriver::getPixelResolution:Wait failed! (timeout is %d ms) %s\n",
                    SAMPLE_READ_WAIT_TIMEOUT, openni::OpenNI::getExtendedError());

        stop();

        return false;
    }

    openni::VideoFrameRef m_frame;

    rc = depth_stream_.readFrame(&m_frame); /*读深度流帧数据*/
    if (rc != openni::STATUS_OK)
    {
        printf("iscas::CameraDriver::getPixelResolution:readFrame failed! %s\n",
               openni::OpenNI::getExtendedError());

        stop();

        return false;
    }

    openni::PixelFormat pixel_for_mat = m_frame.getVideoMode().getPixelFormat();

    if(pixel_for_mat == openni::PIXEL_FORMAT_DEPTH_1_3_MM)
    {
        pixel_resolution = 0.001/3.0;
    }
    else if(pixel_for_mat == openni::PIXEL_FORMAT_DEPTH_1_MM)
    {
        pixel_resolution = 0.001;
    }
    else if(pixel_for_mat != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        pixel_resolution = 0.0001;
    }
    else
    {
        std::printf("iscas::CameraDriver::camera_driver_get_one_frame: Unexpected frame format\n");

        stop();

        return false;
    }

    pixel_resolution_ = pixel_resolution;

    if(!current_is_start)
    {
        stop();
    }
    return true;
}

}
