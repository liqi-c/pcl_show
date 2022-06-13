#include "isca_tof_camera.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#define READ_FROM_XYZ 1

#ifdef READ_FROM_XYZ
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>


typedef struct{
	float x;
	float y;
	float z;
}s_PointData ;

#define PCL_WIDTH 224
#define PCL_HEIGHT 108
#define PCL_FRAME_LENGTH (PCL_WIDTH*PCL_HEIGHT) 

typedef struct ori_data
{
    float x[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云x坐标序列数据起始指针
    float y[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云y坐标序列数据起始指针
    float z[PCL_FRAME_LENGTH]; // = new float///< @brief 输出的点云z坐标序列数据起始指针
}s_xyzdata;

s_xyzdata tmp_data[2] ; 
// 用于测试输出的结果是否正确
bool Convert_PointCloud_To_PCL(iscas::PointCloud& pointcloud_ori, iscas::PointCloud& pointcloud_hdl)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori(new pcl::PointCloud<pcl::PointXYZ>());
    size_t point_size = pointcloud_ori.width * pointcloud_ori.height;
    if(pcl_data_ori->points.size() != point_size)
    {
        pcl_data_ori->points.resize(point_size);
    }
    printf("Ori point size = %ld \n",point_size);
    for(size_t i=0;i<point_size;i++)
    {
        pcl_data_ori->points[i]._PointXYZ::x = pointcloud_ori.x_set[i];
        pcl_data_ori->points[i]._PointXYZ::y = pointcloud_ori.y_set[i];
        pcl_data_ori->points[i]._PointXYZ::z = pointcloud_ori.z_set[i];
		// std::cout<<"ori : i:"<< i << ", x:"<<pcl_data_ori->points[i]._PointXYZ::x <<
        //     ",y:"<<pcl_data_ori->points[i]._PointXYZ::y<<",z:"<<pcl_data_ori->points[i]._PointXYZ::z<<std::endl;
    }
    pcl_data_ori->width = pointcloud_ori.width;
    pcl_data_ori->height = pointcloud_ori.height;
    pcl_data_ori->header.stamp = pointcloud_ori.time_stamp;
    pcl_data_ori->header.frame_id = "camera_link_ori";

	// PointCloud类型转换为pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    point_size = pointcloud_hdl.width * pointcloud_hdl.height;
    if(pcl_data_hdl->points.size() != point_size)
    {
        pcl_data_hdl->points.resize(point_size);
    }
    printf("Hdl point size = %ld \n",point_size);
    for(size_t i=0;i<point_size;i++)
    {
        pcl_data_hdl->points[i]._PointXYZ::x = pointcloud_hdl.x_set[i];
        pcl_data_hdl->points[i]._PointXYZ::y = pointcloud_hdl.y_set[i];
        pcl_data_hdl->points[i]._PointXYZ::z = pointcloud_hdl.z_set[i];
//		std::cout<<"hdl : i:"<< i << ", x:"<<pcl_data_hdl->points[i]._PointXYZ::x <<
//           ",y:"<<pcl_data_hdl->points[i]._PointXYZ::y<<",z:"<<pcl_data_hdl->points[i]._PointXYZ::z<<std::endl;
    }
    pcl_data_hdl->width = pointcloud_hdl.width;
    pcl_data_hdl->height = pointcloud_hdl.height;
    pcl_data_hdl->header.stamp = pointcloud_hdl.time_stamp;
    pcl_data_hdl->header.frame_id = "camera_link_hdl";

	// std::stringstream pcd_filename; 
    // static int pcl_store_index = 0 ;
	// pcd_filename << "pcl_" << pcl_store_index << ".pcd" ; 
	// pcl_store_index++;
	// std::string filename = pcd_filename.str();
	
	// pcl::PCDWriter writer;
	// writer.write(filename,*pcl_data_hdl);
#if 0
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer_pvs(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_pvs->setBackgroundColor (0, 0, 0); // config backend color:dark 
    viewer_pvs->addCoordinateSystem (1.0);// show coordinate xyz 
    viewer_pvs->initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (pcl_data_hdl, 0, 255, 0);
    //viewer_pvs->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "sample cloud");// modify show size 
	//viewer_pvs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "sample_cloud");	//设置点云显示的颜色，rgb 在 [0,1] 范围
	//viewer_pvs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "sample_cloud");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】
    viewer_pvs->addPointCloud<pcl::PointXYZ>(pcl_data_hdl, single_color, "sample cloud");
    while (!viewer_pvs->wasStopped ())
    {
        viewer_pvs->spinOnce();
    }
#else 
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	int v1(0);  //创建左窗口显式cloud1
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("Cloud1", 2, 2, "Cloud1", v1);  //窗口下的标题
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb1(pcl_data_ori, "z");
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data_ori, rgb1, "cloud1", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1", v1);
	viewer.addCoordinateSystem(1.0, "input cloud1", v1);

	int v2(1);  //创建右窗口显示cloud2
	viewer.createViewPort(0.5, 0, 1.0, 1.0, v2);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addText("Cloud2", 2, 2, "Cloud2", v2);  //窗口下的标题
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb2(pcl_data_hdl, "z");
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data_hdl, rgb2, "cloud2", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2", v2);
	viewer.addCoordinateSystem(1.0, "input cloud2", v2);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
#endif 
	return 0 ;
}

bool readSunnyOripcl(const char * file_path, const char *file_pre, int file_index , s_PointData * read_pcl )
{
    char oripcl_filename[128] = { 0 } ; 
    snprintf(oripcl_filename, 128, "%s/%s_%04d.xyz", file_path, file_pre, file_index);
    printf("[Debug]: File name (%s). \n",oripcl_filename);
    FILE *pPCL = fopen(oripcl_filename, "rb");  
    int size = fread(read_pcl,sizeof(s_PointData),PCL_FRAME_LENGTH,pPCL);
    if(0 == size)
    {
        printf("fread file failed.\n");
    }
    fclose(pPCL);
    return true ;
}

bool covertToPointCloud(s_xyzdata * tmp_data ,s_PointData * read_pcl , iscas::PointCloud& pnt_cloud_origin)
{
    for (int i = 0; i < PCL_FRAME_LENGTH; i++)
	{
		//std::cout<<"i:"<< i << ", x:"<<read_pcl[i].x<<",y:"<<read_pcl[i].y<<",z:"<<read_pcl[i].z<<std::endl;
        // 这里考虑的坐标变换
        tmp_data->y[i] = -read_pcl[i].x;
        tmp_data->z[i] = -read_pcl[i].y;
        tmp_data->x[i] = read_pcl[i].z;
	}
    pnt_cloud_origin.x_set = tmp_data->x;
    pnt_cloud_origin.y_set = tmp_data->y;
    pnt_cloud_origin.z_set = tmp_data->z;
    pnt_cloud_origin.size = PCL_FRAME_LENGTH;
    pnt_cloud_origin.width = PCL_WIDTH ;
    pnt_cloud_origin.height = PCL_HEIGHT;
    pnt_cloud_origin.time_stamp = 0;
    // for (int i = 0; i < PCL_FRAME_LENGTH; i++)
    // {
	// 	std::cout<<"ooori - i:"<< i << ", x:"<<pnt_cloud_origin.x_set[i]<<",y:"<<pnt_cloud_origin.y_set[i]<<",z:"<<pnt_cloud_origin.z_set[i]<<std::endl;
    // }

    return 0;
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
        static int index = 35 ;  
        readSunnyOripcl("/home/qli/Desktop/xyz", "ori", index  ,read_pcl);
        covertToPointCloud(&tmp_data[0], read_pcl , pnt_cloud_origin);

        memset(read_pcl,0,sizeof(s_PointData)*PCL_FRAME_LENGTH);
        readSunnyOripcl("/home/qli/Desktop/xyz", "rgb", index  ,read_pcl);
        covertToPointCloud(&tmp_data[1] , read_pcl , pnt_cloud);
        index ++ ;

		if(Convert_PointCloud_To_PCL(pnt_cloud_origin, pnt_cloud_origin)) 
		{
			std::cout << "Convert_PointCloud_To_PCL failed . " << std::endl;
			return -1;
		}
        continue;

        // continue viewer 
	    // pcl::visualization::CloudViewer viewer("pcd viewer");
		// if(Convert_PointCloud_To_PCL(viewer, pnt_cloud_origin)) 
		// {
		// 	std::cout << "Convert_PointCloud_To_PCL failed . " << std::endl;
		// 	return -1;
		// }
        // return 0;
        // continue ; 
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
#ifdef READ_FROM_XYZ
        // 用于测试输出的结果是否正确
		if(Convert_PointCloud_To_PCL(pnt_cloud_origin, pnt_cloud)) 
		{
			std::cout << "Convert_PointCloud_To_PCL failed . " << std::endl;
			return -1;
		}
        // return 0;
#endif         
    }
    //destructor
    iscas::tof_camera_destroy(h);
    return 0;
}
