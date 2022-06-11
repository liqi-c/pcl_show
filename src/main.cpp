#include "isca_tof_camera.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#define READ_FROM_XYZ 1
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

用于测试输出的结果是否正确
bool Convert_PointCloud_To_PCL(pcl::visualization::CloudViewer& viewer, iscas::PointCloud& pointcloud_in)
{
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
		/*
		std::cout << "    " << pcl_data->points[j].x
              << " "    << pcl_data->points[j].y
              << " "    << pcl_data->points[j].z
              << std::endl;
		*/
    }

    pcl_data->width = pointcloud_in.width;
    pcl_data->height = pointcloud_in.height;
    pcl_data->header.stamp = pointcloud_in.time_stamp;
    pcl_data->header.frame_id = "camera_link";

	// std::stringstream pcd_filename; 
	// pcd_filename << "pcl_" << pcl_store_index << ".pcd" ; 
	// pcl_store_index++;
	// std::string filename = pcd_filename.str();
	
	// pcl::PCDWriter writer;
	// writer.write(filename,*pcl_data);
	//pcl::visualization::CloudViewer viewer("pcd viewer");
	viewer.showCloud(pcl_data);

	return 0 ;
}
#ifdef READ_FROM_XYZ

typedef struct{
	float x;
	float y;
	float z;
}s_PointData ;

#define PCL_WIDTH 224
#define PCL_HEIGHT 108
#define PCL_FRAME_LENGTH (PCL_WIDTH*PCL_HEIGHT) 

bool readSunnyOripcl(char * file_path, char *file_pre, int file_index , s_PointData * read_pcl )
{
    char oripcl_filename[128] = { 0 } ; 
 //   s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 } ;
    snprintf(oripcl_filename, 128, "%s/%s_%04d.xyz", file_path, file_pre, file_index);
    printf("[Debug]: File name (%s). \n",oripcl_filename);
    FILE *pPCL = fopen(oripcl_filename, "rb");

    for (int n = 0; n < PCL_FRAME_LENGTH; n++) {
       // fread(read_pcl[n],sizeof(s_PointData),PCL_FRAME_LENGTH,pPCL);
        fread(read_pcl[n],sizeof(s_PointData),1,pPCL);
    }
    fclose(pPCL);
    return true ; 
}

// 下方部分为舜宇点云转换为iscas::PointCloud类型的参考代码，并不是完整的代码
float* x_set = new float[PCL_FRAME_LENGTH]; ///< @brief 输出的点云x坐标序列数据起始指针
float* y_set = new float[PCL_FRAME_LENGTH]; ///< @brief 输出的点云y坐标序列数据起始指针
float* z_set = new float[PCL_FRAME_LENGTH]; ///< @brief 输出的点云z坐标序列数据起始指针
bool covertToPointCloud(s_PointData * read_pcl , iscas::PointCloud& pnt_cloud_origin)
{
    for (int i = 0; i < PCL_FRAME_LENGTH; i++)
	{
		// std::cout<<"x:"<<ff[3*i]<<",y:"<<ff[3*i+1]<<",z:"<<ff[3*i+2]<<std::endl;
        // 这里考虑的坐标变换
        y_set[i] = -read_pcl[i].x;
        z_set[i] = -read_pcl[i].y;
        x_set[i] = read_pcl[i].z;
	}
    pnt_cloud_origin.x_set = x_set;
    pnt_cloud_origin.y_set = y_set;
    pnt_cloud_origin.z_set = z_set;
    pnt_cloud_origin.size = PCL_FRAME_LENGTH;
    pnt_cloud_origin.width = PCL_WIDTH ;

    pnt_cloud_origin.height = PCL_HEIGHT;
    pnt_cloud_origin.time_stamp = 0;
}
#endif 
int main(int argc, char *argv[])
{
    std::cout << "main func has been called!" << std::endl;
    //constructor
    iscas::handle_t h = iscas::tof_camera_create();
    //function call
#ifdef WITH_CAMERA_DRIVER

    if (!iscas::tof_camera_init(h)) {
        std::cerr << "camera init falid!" << std::endl;
        return -1;
    }else {
        std::cout << "camera init successed!" << std::endl;
    }
#endif
    iscas::PointCloud pnt_cloud_origin;
    iscas::PointCloud pnt_cloud;
    iscas::DepthImage image;
    iscas::PoseStamped pose;

    // // ------------------------设置参数，如果只使用pcl变换，则必须设置-------------------------------------------
    iscas::Param param;
    param.max_depth = 1.2;
    param.min_depth = 0;
    param.max_hight = 0.1;
    param.min_hight = 0;

    param.camera_offset = 0.04; //相机在地面的高度
    param.size_leaf = 0.005; //下采样参数    
    //设置相机的参数,这个axon保持默认
    param.camera_param.scaling = 0.000333333;
    param.camera_param.camera_cf.fx = 507.075;
    param.camera_param.camera_cf.fy = 506.05;
    param.camera_param.camera_cf.cx = 306.375;
    param.camera_param.camera_cf.cy = 227.914;
    //设置默认镜头到相机坐标系变换关系,这个axon保持默认
    param.camera_param.camera_tf.x = 0;
    param.camera_param.camera_tf.y = -0.0055f;
    param.camera_param.camera_tf.z = 0;
    param.camera_param.camera_tf.yaw = -1.570796326794896619231321691639751442f;
    param.camera_param.camera_tf.pitch = 0;
    param.camera_param.camera_tf.roll = -1.570796326794896619231321691639751442f;

    iscas::tof_camera_set_paramters(h, param);
    // // ------------------------测试设置参数是否正确--------------------------------------------------------------------

#ifdef WITH_CAMERA_DRIVER
	// pcl::visualization::CloudViewer viewer("pcd viewer");
    if (!iscas::tof_camera_start(h)) {
        std::cerr << "start camera failed!" << std::endl;
        return -1;
    }
#endif
    while (1) {
 
#ifdef READ_FROM_XYZ
  // read from xyz file . 
        s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 } ;  
        static int index = 0 ;  
        readSunnyOripcl("/mnt/hgfs/ToF_DATA/20220607/20220607_2/pcl", "ori", index  ,&read_pcl);
        index ++ ;
        covertToPointCloud(&read_pcl , pnt_cloud_origin);

        // continue viewer 
	    pcl::visualization::CloudViewer viewer("pcd viewer");
		if(Convert_PointCloud_To_PCL(viewer, pnt_cloud)) 
		{
			std::cout << "Convert_PointCloud_To_PCL failed . " << std::endl;
			return -1;
		}
        continue ; 
#endif 

#ifdef WITH_CAMERA_DRIVER
        // bool ret = iscas::tof_camera_get_obstcale_point_cloud(h, pnt_cloud, image, pose);
        // if (false == ret) {
        //     std::cout << "get one frame failed!" << std::endl;
        //     continue;
        // }
        // std::cout << "get one frame, data size: " << pnt_cloud.size << std::endl;

        // 调用深度相机获取一帧
        bool ret = iscas::tof_camera_get_one_frame_depth_image(h, image);
        if (false == ret) {
            std::cout << "Get one frame failed!" << std::endl;
            continue;
        }
#endif
        // 调用pcl直接处理一帧点云
        bool ret1 = iscas::tof_camera_cvt_point_cloud_to_point_cloud(h, pnt_cloud_origin, pnt_cloud, pose);

        // 调用pcl处理一帧图像
        // bool ret1 = iscas::tof_camera_cvt_depth_image_to_point_cloud(h, image, pnt_cloud, pose);

        // 使用深度图直接转换为点云
        // bool ret1 = iscas::tof_camera_cvt_depth_image_to_point_cloud_origin(h, image, pnt_cloud);
        
        if (false == ret1) {
            std::cout << "Convert one frame failed!" << std::endl;
            continue;
        }

        // 用于测试输出的结果是否正确
		if(Convert_PointCloud_To_PCL(viewer, pnt_cloud)) 
		{
			std::cout << "Convert_PointCloud_To_PCL failed . " << std::endl;
			return -1;
		}
    }
    //destructor
    iscas::tof_camera_destroy(h);
    return 0;
}
