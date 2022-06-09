#include "depth_image_to_pointcloud.h"
#include <cmath>
#include<limits>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件

#include <pcl/filters/extract_indices.h>//通过序号过滤点云
#include <pcl/filters/radius_outlier_removal.h>  //半径滤波
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波
#include <pcl/filters/approximate_voxel_grid.h>//approximate体素滤波
#include <pcl/segmentation/sac_segmentation.h>//平面提取

// #include <png.h>

// #define PRINT_TIME 0

// #include <time.h>
#include <sys/time.h>
#include <sstream>

namespace iscas
{

DepthImageToPointCloud::DepthImageToPointCloud():
    is_get_cf_(false),
    is_get_tf_(false),
    is_get_scaling_(false),
    width_(0),
    height_(0)
{
    pc_.x_set = nullptr;
    pc_.y_set = nullptr;
    pc_.z_set = nullptr;
    pc_.size = 0;
    pc_.width = 0;
    pc_.height = 0;

    x_set = nullptr;
    y_set = nullptr;
    z_set = nullptr;

    float ini_value[4] = {0, 0, 1, 0.04};
    model_coefficients_last_.values.insert(model_coefficients_last_.values.begin(), ini_value, ini_value+5);

    min_x_ = 0.0; ///<@brief 最小深度，默认0 单位m
    max_x_ = 1.2; ///<@brief 最大深度，默认1.2 单位m
    min_z_ = -0.1; ///<@brief 最小高度，默认-0.1 单位m
    max_z_ = 0.08; ///<@brief 最大高度，默认0.08 单位m

    size_leaf_ = 0.005;///<@brief 点云下采样参数
    camera_offset_ = 0.04;///<@brief 相机的距离地面的高度

#ifdef PCL_VISUALIZOR
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”
	viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));

	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer_->setBackgroundColor (0, 0, 0); 

	//添加坐标系
	viewer_->addCoordinateSystem(1.0);

	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer_->initCameraParameters ();

	//设置相机角度
	viewer_->setCameraPosition(-1, 0, 0, 0, 0, 1, 0);

    single_color_r_ = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(255, 0, 0);
    single_color_g_ = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(0, 255, 0);
    single_color_b_ = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(0, 0, 255);
    single_color_w_ = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(255, 255, 255);

#endif

}

DepthImageToPointCloud::~DepthImageToPointCloud()
{
    if(pc_.x_set != nullptr)
    {
        delete [] pc_.x_set;
    }

    if(pc_.y_set != nullptr)
    {
        delete [] pc_.y_set;
    }

    if(pc_.z_set != nullptr)
    {
        delete [] pc_.z_set;
    }

    if(x_set != nullptr)
    {
        delete [] x_set;
    }

    if(y_set != nullptr)
    {
        delete [] y_set;
    }

    if(z_set != nullptr)
    {
        delete [] z_set;
    }
}

bool DepthImageToPointCloud::setCF(const CameraCF cf)
{
    if(!is_get_scaling_)
    {
        std::printf("Please set the resolution first.call setUnitScaling().\r\n");
        return false;
    }

    cf_ = cf;

    constant_x_ = unit_scaling_ / cf_.fx;
    constant_y_ = unit_scaling_ / cf_.fy;
    center_x_ = cf_.cx;
    center_y_ = cf_.cy;

    is_get_cf_ = true;

    return true;
}

bool DepthImageToPointCloud::setUnitScaling(const double scaling)
{
    unit_scaling_ = scaling;
    is_get_scaling_ = true;
    return true;
}

bool DepthImageToPointCloud::setTF(const CameraTF tf)
{

    float theta_x = tf.roll;
    float Rx[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    Rx[0][0] = 1.0;
    Rx[1][1] = std::cos(theta_x);
    Rx[1][2] = -std::sin(theta_x);
    Rx[2][1] = std::sin(theta_x);
    Rx[2][2] = std::cos(theta_x);

    float theta_y = tf.pitch;
    float Ry[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    Ry[0][0] = std::cos(theta_y);
    Ry[0][2] = std::sin(theta_y);
    Ry[1][1] = 1.0;
    Ry[2][0] = -std::sin(theta_y);
    Ry[2][2] = std::cos(theta_y);

    float theta_z = tf.yaw;
    float Rz[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    Rz[0][0] = std::cos(theta_z);
    Rz[0][1] = -std::sin(theta_z);
    Rz[1][0] = std::sin(theta_z);
    Rz[1][1] = std::cos(theta_z);
    Rz[2][2] = 1.0;

    float Rzy[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++)
        {
            Rzy[i][j] = Rz[i][0]*Ry[0][j] + Rz[i][1]*Ry[1][j] + Rz[i][2]*Ry[2][j];
        }
    }

    float Rzyx[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++)
        {
            Rzyx[i][j] = Rzy[i][0]*Rx[0][j] + Rzy[i][1]*Rx[1][j] + Rzy[i][2]*Rx[2][j];
        }
    }

    matrix4f_[0][0] = Rzyx[0][0];
    matrix4f_[0][1] = Rzyx[0][1];
    matrix4f_[0][2] = Rzyx[0][2];

    matrix4f_[1][0] = Rzyx[1][0];
    matrix4f_[1][1] = Rzyx[1][1];
    matrix4f_[1][2] = Rzyx[1][2];

    matrix4f_[2][0] = Rzyx[2][0];
    matrix4f_[2][1] = Rzyx[2][1];
    matrix4f_[2][2] = Rzyx[2][2];

    matrix4f_[0][3] = tf.x;
    matrix4f_[1][3] = tf.y;
    matrix4f_[2][3] = tf.z;
    matrix4f_[3][3] = 1.0;

    is_get_tf_ = true;
    return true;
}

bool DepthImageToPointCloud::transformDepthImageToPointCloud(const DepthImage &image_in, PointCloud &pointcloud_out)
{
    if(!is_get_cf_)
    {
        std::printf("error,please call setCF().\r\n");
        return false;
    }

    if(image_in.width != width_ || image_in.height != height_)
    {
        width_ = image_in.width;
        height_ = image_in.height;

        if(pc_.x_set != nullptr)
        {
            delete [] pc_.x_set;
        }

        if(pc_.y_set != nullptr)
        {
            delete [] pc_.y_set;
        }

        if(pc_.z_set != nullptr)
        {
            delete [] pc_.z_set;
        }

        pc_.x_set = new float[width_*height_];
        pc_.y_set = new float[width_*height_];
        pc_.z_set = new float[width_*height_];
        pc_.size = image_in.size;
        pc_.width = width_;
        pc_.height = height_;
    }

    const uint16_t* pDepth = reinterpret_cast<const uint16_t*>(image_in.data_set); /*当前帧的深度数据*/

    float bad_point = std::numeric_limits<float>::quiet_NaN();
    float px,py,pz;
    float V[4] = {0,0,0,0};
    int frame_drop_index = 50;//设置图像剪裁的边缘的像素
    int height_end = height_ - frame_drop_index;
    int width_end = width_ - frame_drop_index;
    for (int v = 0; v < height_; ++v)
    {
        for (int u = 0; u < width_; ++u)
        {
            unsigned int index = static_cast<unsigned int>(v*width_+u);
            uint16_t depth = pDepth[index];

            if (depth == 0 || depth == 4096 || v < frame_drop_index || u < frame_drop_index || v >= height_end || u >= width_end)
            {
                pc_.x_set[index] = bad_point;
                pc_.y_set[index] = bad_point;
                pc_.z_set[index] = bad_point;
            }
            else
            {
                px = (u - center_x_) * depth * constant_x_;
                py = (v - center_y_) * depth * constant_y_;
                pz = static_cast<float>(depth*unit_scaling_);

                if(is_get_tf_)
                {
                    V[0] = matrix4f_[0][0]*px + matrix4f_[0][1]*py + matrix4f_[0][2]*pz + matrix4f_[0][3]*1.0f;
                    V[1] = matrix4f_[1][0]*px + matrix4f_[1][1]*py + matrix4f_[1][2]*pz + matrix4f_[1][3]*1.0f;
                    V[2] = matrix4f_[2][0]*px + matrix4f_[2][1]*py + matrix4f_[2][2]*pz + matrix4f_[2][3]*1.0f;

                    pc_.x_set[index] = V[0];
                    pc_.y_set[index] = V[1];
                    pc_.z_set[index] = V[2];
                }
                else
                {
                    pc_.x_set[index] = px;
                    pc_.y_set[index] = py;
                    pc_.z_set[index] = pz;
                }
            }
        }
    }

    pc_.time_stamp = image_in.time_stamp;

    pointcloud_out = pc_;

    return true;
}

bool DepthImageToPointCloud::setLimits(double min_x, double max_x, double min_z, double max_z)
{
    min_x_ = min_x;
    max_x_ = max_x;
    min_z_ = min_z;
    max_z_ = max_z;
    printf("min_x: %f, max_x: %f, min_z: %f, max_z: %f\n", min_x_, max_x_, min_z_, max_z_);
    return true;
}

#ifdef PCL_VISUALIZOR
/*******可视化单个点云：应用PCLVisualizer可视化类显示单个具有XYZ信息的点云***********/
//showCloud函数实现最基本的点云可视化操作，
void DepthImageToPointCloud::showCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
										pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string color, double point_size, std::string id)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>* single_color; // green
	if(color == "r")
	{
		single_color = single_color_r_;
	}
	else if(color == "g")
	{
		single_color = single_color_g_;
	}
	else if(color == "b")
	{
		single_color = single_color_b_;
	}
	else
	{
		single_color = single_color_w_;
	}
	single_color->setInputCloud(cloud);

	if(!viewer->updatePointCloud<pcl::PointXYZ> (cloud, *single_color, id))
	{
		printf("ADD POINTCLOUD\n");
		viewer->addPointCloud<pcl::PointXYZ> (cloud, *single_color, id); 
	}
	
	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法,1设置显示点云大小
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);

}
#endif

bool DepthImageToPointCloud::processPointCloud(const PointCloud &pointcloud_in, PointCloud &pointcloud_out)
{
    if(!is_get_cf_)
    {
        std::printf("error,please call setCF().\r\n");
        return false;
    }

#ifdef PRINT_TIME
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double start_time = tv.tv_sec + tv.tv_usec / 1e6, current_time;
    printf("start_time: %lf\n", start_time);
#endif

    // PointCloud类型转换为pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data(new pcl::PointCloud<pcl::PointXYZ>());

    size_t point_size = pointcloud_in.width * pointcloud_in.height;

    if(pcl_data->points.size() != point_size)
    {
        pcl_data->points.resize(point_size);
    }

    for(size_t i=0;i<point_size;i++)
    {
        pcl_data->points[i]._PointXYZ::x = pointcloud_in.x_set[i];
        pcl_data->points[i]._PointXYZ::y = pointcloud_in.y_set[i];
        pcl_data->points[i]._PointXYZ::z = pointcloud_in.z_set[i];
    }

    pcl_data->width = pointcloud_in.width;
    pcl_data->height = pointcloud_in.height;
    pcl_data->header.stamp = pointcloud_in.time_stamp;
    pcl_data->header.frame_id = "camera_link";

/************************  点云处理*************************************/

    // 不使用条件滤波，直接使用for循环判断过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    double max_z = 0.15-camera_offset_;
    double min_z = -0.1-camera_offset_;
    for (const auto& point: pcl_data->points) 
    {
        if( point.x > min_x_ && point.x < max_x_ && point.z > min_z && point.z < max_z )
        {
            cloud_filtered->push_back( point );
        }
    }

    //降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr after_down_sampling(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter_voxel;
    filter_voxel.setInputCloud(cloud_filtered);
    filter_voxel.setLeafSize(size_leaf_, size_leaf_, size_leaf_);
    filter_voxel.setDownsampleAllData(false);
    filter_voxel.filter(*after_down_sampling);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs_raw(new pcl::PointCloud<pcl::PointXYZ>());//用于保存点云中去掉地面后的障碍物信息
    *cloud_obs_raw = *after_down_sampling;
    bool find_ground_flag = false;
    pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients());
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_plane; 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


    if(!after_down_sampling->empty())
    {
        // estimate the plane model parameters together with the inlier markings;
        pcl::SACSegmentation<pcl::PointXYZ> seg_solver;
        seg_solver.setInputCloud(after_down_sampling);
        seg_solver.setOptimizeCoefficients(true);
        seg_solver.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg_solver.setMethodType(pcl::SAC_RANSAC);
        seg_solver.setMaxIterations(200);
        seg_solver.setDistanceThreshold(0.01);
        //because we want a specific plane (X-Y Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
        // seg_solver.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
        if(std::abs(model_coefficients_last_.values[2]) > 0.95)
        {
            seg_solver.setAxis(Eigen::Vector3f(model_coefficients_last_.values[0], model_coefficients_last_.values[1],model_coefficients_last_.values[2]));
        }
        else
        {
            seg_solver.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
        }
        
        seg_solver.setEpsAngle(pcl::deg2rad(10.0)); // plane can be within 7.0 degrees of X-Z plane

        seg_solver.segment(*inliers, *model_coefficients);
        // std::cout << *model_coefficients << std::endl;

        if(inliers->indices.size() > 3)
        {
            extract_indices_plane.setInputCloud(after_down_sampling);
            extract_indices_plane.setIndices(inliers);
            if(model_coefficients->values[3] > 0.03)
            {
                extract_indices_plane.setNegative(true);
                extract_indices_plane.filter(*cloud_obs_raw);
                find_ground_flag = true;
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_top(new pcl::PointCloud<pcl::PointXYZ>());
                extract_indices_plane.setNegative(false);
                extract_indices_plane.filter(*cloud_plane_top);
                extract_indices_plane.setNegative(true);
                extract_indices_plane.filter(*cloud_obs_raw);
                seg_solver.setInputCloud(cloud_obs_raw);
                seg_solver.segment(*inliers, *model_coefficients);

                if((inliers->indices.size() > 3) && (model_coefficients->values[3] > 0.04))
                {
                    // 找到了地面
                    extract_indices_plane.setInputCloud(cloud_obs_raw);
                    extract_indices_plane.setIndices(inliers);
                    extract_indices_plane.setNegative(true);
                    extract_indices_plane.filter(*cloud_obs_raw);
                    find_ground_flag = true;
                }
                
                *cloud_obs_raw = (*cloud_obs_raw) + (*cloud_plane_top);
            }
        }
    }

    if(find_ground_flag)
    {
        model_coefficients_last_ = *model_coefficients;
        // printf("cloud_obs_raw->size(): %d\n", (int)cloud_obs_raw->size());
        inliers->indices.clear();
        for(int tempi = 0; tempi < cloud_obs_raw->size(); ++tempi)
        {
            double result = cloud_obs_raw->points[tempi].x * model_coefficients->values[0]
                            + cloud_obs_raw->points[tempi].y * model_coefficients->values[1]
                            + cloud_obs_raw->points[tempi].z * model_coefficients->values[2]
                            + model_coefficients->values[3];
            if(((model_coefficients->values[2] > 0) && (result < 0.005))
            || ((model_coefficients->values[2] < 0) && (result > 0.005)))
            {
                // printf("(%d) result < 0\n", tempi);
                inliers->indices.push_back(tempi);
            }
        }
        // std::cout << *inliers << std::endl;
        extract_indices_plane.setInputCloud(cloud_obs_raw);
        extract_indices_plane.setIndices(inliers);
        extract_indices_plane.setNegative(true);
        extract_indices_plane.filter(*cloud_obs_raw);
    }
    else
    {
        inliers->indices.clear();

        for(int tempi = 0; tempi < cloud_obs_raw->size(); ++tempi)
        {
            double result = cloud_obs_raw->points[tempi].x * model_coefficients_last_.values[0]
                            + cloud_obs_raw->points[tempi].y * model_coefficients_last_.values[1]
                            + cloud_obs_raw->points[tempi].z * model_coefficients_last_.values[2]
                            +  model_coefficients_last_.values[3];
            if(((model_coefficients_last_.values[2] > 0) && (result < 0.01))
            || ((model_coefficients_last_.values[2] < 0) && (result > 0.01)))
            {
                // printf("(%d) result < 0\n", tempi);
                inliers->indices.push_back(tempi);
            }
        }
        // std::cout << *inliers << std::endl;
        extract_indices_plane.setInputCloud(cloud_obs_raw);
        extract_indices_plane.setIndices(inliers);
        extract_indices_plane.setNegative(true);
        extract_indices_plane.filter(*cloud_obs_raw);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_naive_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    double max_z2 = max_z_ - camera_offset_;
    double min_z2 = min_z_ - camera_offset_;
    for (const auto& point: cloud_obs_raw->points) 
    {
        if(point.z > min_z2 && point.z < max_z2 )
        {
            cloud_naive_filtered->push_back( point );
        }
    }

    if(cloud_naive_filtered->size() > 0)
    {
        // 统计滤波器，比较耗时
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_naive_filtered);
        sor.setMeanK (30);
        sor.setStddevMulThresh (0.5);
        sor.filter (*cloud_obs_raw);
    }

#ifdef PCL_VISUALIZOR
    showCloud(viewer_, pcl_data, "w", 1, "original");
    // showCloud(viewer_, cloud_obstacle_base_slice, "r", 1, "cloud_obstacle_base_slice");
    showCloud(viewer_, after_down_sampling, "g", 3, "after_down_sampling");
    showCloud(viewer_, cloud_obs_raw, "b", 5, "cloud_obs_raw");

    viewer_->spinOnce();//用于刷新显示
#endif

    // pcl::PointCloud转换为PointCloud
    point_size = cloud_obs_raw->width * cloud_obs_raw->height;
    // printf("pointcloud_out.size(%d) != point_size(%d)\n", pointcloud_out.size, point_size);
    if(x_set != nullptr)
    {
        delete [] x_set;
    }

    if(y_set != nullptr)
    {
        delete [] y_set;
    }

    if(z_set != nullptr)
    {
        delete [] z_set;
    }

    x_set = new float[point_size];
    y_set = new float[point_size];
    z_set = new float[point_size];

    pointcloud_out.x_set = x_set;
    pointcloud_out.y_set = y_set;
    pointcloud_out.z_set = z_set;
    pointcloud_out.size = point_size;
    pointcloud_out.width = cloud_obs_raw->width;
    pointcloud_out.height = cloud_obs_raw->height;
    pointcloud_out.time_stamp = pointcloud_in.time_stamp;

    for(size_t i=0;i<point_size;i++)
    {
        pointcloud_out.x_set[i] = cloud_obs_raw->points[i]._PointXYZ::x;
        pointcloud_out.y_set[i] = cloud_obs_raw->points[i]._PointXYZ::y;
        pointcloud_out.z_set[i] = cloud_obs_raw->points[i]._PointXYZ::z;
    }
    // printf("pointcloud_out.size: %d\n", pointcloud_out.size);
#ifdef PRINT_TIME
    gettimeofday(&tv, NULL);
    current_time = tv.tv_sec + tv.tv_usec / 1e6;
    printf("End pointcloud process: %lf ms\n", (current_time - start_time)*1e3);
    start_time = tv.tv_sec + tv.tv_usec / 1e6;
#endif

    return true;
}

bool DepthImageToPointCloud::setParam(Param param)
{
    if(!setUnitScaling(param.camera_param.scaling))
    {
        std::printf("[Error]Call setUnitScaling() fail.\r\n");
        return false;
    }
    if(!setTF(param.camera_param.camera_tf))
    {
        std::printf("[Error]Call setTF() fail.\r\n");
        return false;
    }
    if(!setCF(param.camera_param.camera_cf))
    {
        std::printf("[Error]Call setCF() fail.\r\n");
        return false;
    }
    if(!setLimits(param.min_depth, param.max_depth, param.min_hight, param.max_hight))
    {
        std::printf("[Error]Call setLimits() fail.\r\n");
        return false;
    }
    size_leaf_ = param.size_leaf;
    camera_offset_ = param.camera_offset;
    printf("size_leaf: %f, camera_offset: %f\n", size_leaf_, camera_offset_);
    return true;
}
}

