
#include "pcl_show.hpp"

s_xyzdata tmp_data[2] ;

//int CameraPose[7]={0,0,2,0,2,0,0};
int CameraPose[7]={-2,0,1,2,0,0,0};

// （pos_x,pos_y,pos_z）为相机光心，
//  (view_x, view_y, view_z)为光轴上的一点，他们的差向量决定了光轴方向，
//  (up_x, up_y, up_z)确定相机的正上方

// 0 0 0 0 1 0 0 0 2 0 ### 过于大 ，缩小后视角是俯视试图

// 0 0 2 0 2 0 0 ## z轴看不到，y 朝上，X水平
// 0 0 2 2 0 0 0 ## z轴看不到，x 朝上，y水平
// -2 0 2 2 0 0 0 ## x,z轴夹角面向屏幕，y水平
// -2 0 1 2 0 0 0 ## x,z轴夹角面向屏幕，y水平

// 		## 前面3位，相当于是相机的位置，只有一位有值，则相机在轴线上，看不到对应的轴，如果2位有值，则相机在这两个轴线的中间夹角45度，有值的两个方向成90度夹角面对屏幕
// 		## 中间3位，视角方向，哪个有值，则看向哪个轴（面），有值的那个轴（面）朝上
//    viewer_pvs.addCoordinateSystem (0.1);    // 参数越大，坐标轴越宽


bool readSunnyXyz(const char * file , s_PointData *read_pcl)
{
    printf("[Debug]: File name (%s). \n",file);
    FILE *pPCL = fopen(file, "rb");
    int size = fread(read_pcl, sizeof(s_PointData), PCL_FRAME_LENGTH, pPCL);
    if(0 == size)
    {
        printf("fread file failed.\n");
    }
    fclose(pPCL);
    return true ;
}
bool Read_Pcd(const char * file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *pointcloud_ori) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
    return true ;
}
bool Convert_SunnyXyz_To_IscasPCL(s_xyzdata * tmp_data ,s_PointData * read_pcl , iscas::PointCloud& pnt_cloud_origin)
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
bool Convert_PCL_To_IscasPCL( pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data , iscas::PointCloud& pointcloud_ori )
{
    if(pcl_data->points.size() != pointcloud_ori.width * pointcloud_ori.height)
    {
        pointcloud_ori.width = pcl_data->width;
        pointcloud_ori.height = pcl_data->height;
    }
    printf("Ori point size = %ld \n", pcl_data->points.size());
    for(size_t i=0 ; i < pcl_data->points.size() ; i++)
    {
        pointcloud_ori.x_set[i] = pcl_data->points[i]._PointXYZ::x;
        pointcloud_ori.y_set[i] = pcl_data->points[i]._PointXYZ::y;
        pointcloud_ori.z_set[i] = pcl_data->points[i]._PointXYZ::z;
    }
    pointcloud_ori.time_stamp = pcl_data->header.stamp;
    return 0 ;
}
bool Convert_IscasPCL_To_PCL(iscas::PointCloud& pointcloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori)
{
   // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori(new pcl::PointCloud<pcl::PointXYZ>());
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
    return 0 ;
}
bool PCL_Show_Signle(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);

 //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_data, 255, 0, 0);
	//viewer.addPointCloud(pcl_data, "cloud_ori");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgbz(pcl_data, "z"); // 配置沿着Z轴的颜色变化
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data, rgbz, "cloud_ori");

	viewer.addCoordinateSystem(0.1);

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_ori");// modify show size

	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "cloud_ori");//红色，设置点云显示的颜色，rgb 在 [0,1] 范围
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud_ori");//黑色，设置点云显示的颜色，rgb 在 [0,1] 范围

//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud_ori");		//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】
//	viewer.initCameraParameters();

    //viewer_pvs->spinOnce(100);  // 100ms
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
	return 0 ;
}
bool PCL_Show_Double_In_1_Windows(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_hdl)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
   // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_data_ori, 255, 255, 0);// yellow
	viewer.addPointCloud(pcl_data_ori, cloud_tr_color_h, "cloud_ori");
//	viewer.addPointCloud(pcl_data_ori,  "cloud_ori");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(pcl_data_hdl, 255, 0,0); // red
	viewer.addPointCloud(pcl_data_hdl, cloud_icp_color_h, "cloud_hdl");
//	viewer.addPointCloud(pcl_data_hdl,  "cloud_ori");

	viewer.addCoordinateSystem(0.0);
	viewer.initCameraParameters();

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

    //viewer_pvs->spinOnce(100);  // 100ms
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
	return 0 ;
}
bool PCL_Show_Double_In_2_Windows(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_hdl)
{
    // PointCloud类型转换为pcl::PointCloud
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

	int v1(0);  //创建左窗口显式cloud1
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("Cloud1", 2, 2, "Cloud1", v1);  //窗口下的标题
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb1(pcl_data_ori, "z");
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data_ori, rgb1, "cloud1", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1", v1);
	viewer.addCoordinateSystem(0.1, "input cloud1", v1);

	int v2(1);  //创建右窗口显示cloud2
	viewer.createViewPort(0.5, 0, 1.0, 1.0, v2);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addText("Cloud2", 2, 2, "Cloud2", v2);  //窗口下的标题
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb2(pcl_data_hdl, "z");
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data_hdl, rgb2, "cloud2", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2", v2);
	viewer.addCoordinateSystem(0.1, "input cloud2", v2);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
	return 0 ;
}

bool PCL_Show_Continue_One(pcl::visualization::PCLVisualizer viewer_pvs,pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data)
{
    static int init_flag = 0;

    viewer_pvs.setBackgroundColor (0, 0, 0); // config backend color:dark
    viewer_pvs.addCoordinateSystem (0.1);    // show coordinate xyz

    viewer_pvs.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

   // viewer_pvs.initCameraParameters ();

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgbz(pointcloud_data, "z"); // 配置沿着Z轴的颜色变化
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (pointcloud_data, 0, 255, 0);
    if(1 != init_flag) // add once .
    {
        viewer_pvs.addPointCloud<pcl::PointXYZ>(pointcloud_data, rgbz, "pointcloud_data");
    }

    viewer_pvs.updatePointCloud(pointcloud_data, rgbz, "pointcloud_data");

    viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pointcloud_data");// modify show size
	//viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "pointcloud_data");	//设置点云显示的颜色，rgb 在 [0,1] 范围
	//viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "pointcloud_data");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】

    viewer_pvs.spinOnce(1000);  // 100ms
    init_flag = 1;
    return 0;
}
bool PCL_Show_Continue_Double(pcl::visualization::PCLVisualizer viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    static int init_flag = 0;
    viewer.setBackgroundColor (0, 0, 0); // config backend color:dark
    viewer.addCoordinateSystem (0.1);    // show coordinate xyz

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

	static int v1(0);  //创建左窗口显式cloud1
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb1(cloud1, "z");

    if(1 != init_flag)
	{
        viewer.createViewPort(0, 0, 0.5, 1.0, v1);  //左右窗口大小划分，1:1
	    viewer.setBackgroundColor(0, 0, 0, v1);
    //	viewer.addText("Cloud1", 2, 2, "Cloud1", v1);  //窗口下的标题
        viewer.addPointCloud<pcl::PointXYZ>(cloud1, rgb1, "cloud1", v1);
    }
    viewer.updatePointCloud(cloud1, rgb1, "cloud1");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1", v1);
//	viewer.addCoordinateSystem(0.1, "input cloud1", v1);

	static int v2(1);  //创建右窗口显示cloud2
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb2(cloud2, "z");

    if(1 != init_flag)
	{
        viewer.createViewPort(0.5, 0, 1.0, 1.0, v2);  //左右窗口大小划分，1:1
        viewer.setBackgroundColor(0, 0, 0, v2);
    //	viewer.addText("Cloud2", 2, 2, "Cloud2", v2);  //窗口下的标题
	    viewer.addPointCloud<pcl::PointXYZ>(cloud2, rgb2, "cloud2", v2);
    }
    viewer.updatePointCloud(cloud2, rgb2, "cloud2");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2", v2);
//	viewer.addCoordinateSystem(0.1, "input cloud2", v2);

    viewer.spinOnce(1000);  // 100ms
 //   viewer.removeAllPointClouds();  // 移除当前所有点云
 //   viewer.removeAllTexts();  // 移除当前所有点云
    init_flag = 1;

    return 0;
}

bool Read_PCD_and_Show_Single(const char * life_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());

    Read_Pcd(life_path , pointcloud_ori);
    if(PCL_Show_Signle( pointcloud_ori ))
    {
        std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
        return -1;
    }
    return 0;
}

bool Read_PCD_and_Show_Double(const char * life_path,const char * right_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    Read_Pcd(life_path , pointcloud_ori);
    Read_Pcd(right_path , pointcloud_hdl);

    if(PCL_Show_Double_In_2_Windows( pointcloud_ori, pointcloud_hdl))
    {
        std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
        return -1;
    }
    return 0;
}

bool Read_PCD_and_Show_Continue_Single(const char * life_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::visualization::PCLVisualizer viewer_pvs("Cloud Viewer");

    DIR *o_dir;
    struct dirent *o_ptr;
	char ori_file_name[MAX_FILENAME_LENGTH]={};

    const char * OribasePath = life_path;

    if ((o_dir=opendir(OribasePath)) == NULL )
    {
        printf("Open dir error...%s.\n",OribasePath);
        exit(1);
    }
    while ((o_ptr=readdir(o_dir)) != NULL)
    {
        if(strcmp(o_ptr->d_name,".")==0 || strcmp(o_ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(o_ptr->d_type == 8)    ///file
        {
            sprintf((char *)ori_file_name,"%s/%s",OribasePath,o_ptr->d_name);
            printf("File_name:%s\n",ori_file_name);
        }
        Read_Pcd(ori_file_name , pointcloud_ori);
        if(PCL_Show_Continue_One(viewer_pvs, pointcloud_ori ))
        {
            std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
            return -1;
        }
    }
    return 0;
}
bool Read_PCD_and_Show_Continue_Double(const char * life_path, const char * right_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::visualization::PCLVisualizer viewer_pvs("Cloud Viewer");

    DIR *o_dir;    DIR *h_dir;
    struct dirent *o_ptr;    struct dirent *h_ptr;
	char ori_file_name[MAX_FILENAME_LENGTH]={};
	char hdl_file_name[MAX_FILENAME_LENGTH]={};

    const char * OribasePath = life_path;
    const char * HdlbasePath = right_path;

    if ((o_dir=opendir(OribasePath)) == NULL || (h_dir=opendir(HdlbasePath)) == NULL)
    {
        printf("Open dir error...%s-%s.\n",OribasePath,HdlbasePath);
        exit(1);
    }
    do{

        while ((o_ptr=readdir(o_dir)) != NULL)
        {
            if(strcmp(o_ptr->d_name,".")==0 || strcmp(o_ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
            else if(o_ptr->d_type == 8)    ///file
            {
                sprintf((char *)ori_file_name,"%s/%s",OribasePath,o_ptr->d_name);
                printf("ori_file_name:%s\n",ori_file_name);
            }
            Read_Pcd(ori_file_name , pointcloud_ori);
            break;
        }
        while ((h_ptr=readdir(h_dir)) != NULL)
        {
            if(strcmp(h_ptr->d_name,".")==0 || strcmp(h_ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
            else if(h_ptr->d_type == 8)    ///file
            {
                sprintf((char *)hdl_file_name,"%s/%s",HdlbasePath,h_ptr->d_name);
                printf("hdl_file_name:%s\n",hdl_file_name);
            }
            Read_Pcd(hdl_file_name , pointcloud_hdl);
            break;
        }
        if(o_ptr != NULL && NULL != h_ptr)
        {
            if(PCL_Show_Continue_Double(viewer_pvs, pointcloud_ori, pointcloud_hdl ))
            {
                std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
                return -1;
            }
        }else{
            break;
        }
    }while(1);
    return 0;
}

int test_alg(void)
{
    //constructor
    iscas::handle_t h = iscas::tof_camera_create();
    iscas::PointCloud pnt_cloud_origin;
    iscas::PointCloud pnt_cloud;
    // iscas::DepthImage image;
    iscas::PoseStamped pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::visualization::PCLVisualizer viewer_pvs("Cloud Viewer");

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

    while (1)
    {
#if  0
        s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 };
        static int index = 25 ;
        readSunnyXyz("/home/liq/CODE/ori_16.xyz" , read_pcl);
        Convert_SunnyXyz_To_IscasPCL(&tmp_data[0], read_pcl , pnt_cloud_origin);
#else
        Read_Pcd("/home/liq/CODE/ori_16.pcd" , pointcloud_ori);

        pnt_cloud_origin.x_set = tmp_data[0].x;
        pnt_cloud_origin.y_set = tmp_data[0].y;
        pnt_cloud_origin.z_set = tmp_data[0].z;

        Convert_PCL_To_IscasPCL(pointcloud_ori, pnt_cloud_origin);

#endif
        bool ret1 = iscas::tof_camera_cvt_point_cloud_to_point_cloud(h, pnt_cloud_origin, pnt_cloud, pose);

        if (false == ret1) {
            std::cout << "Convert one frame failed!" << std::endl;
			return -1;
        }

       // 用于测试输出的结果是否正确
        Convert_IscasPCL_To_PCL(pnt_cloud_origin , pointcloud_ori);
        Convert_IscasPCL_To_PCL(pnt_cloud , pointcloud_hdl);

        //if(PCL_Show_Double_In_1_Windows(pointcloud_ori, pointcloud_hdl))
        //if(PCL_Show_Double_In_2_Windows(pointcloud_ori, pointcloud_hdl))
        //if(PCL_Show_Signle( pointcloud_ori))
        {
 			std::cout << "PCL_Show_Double_In_2_Windows failed . " << std::endl;
			return -1;
        }
    }
    //destructor
    iscas::tof_camera_destroy(h);
    return 0;
}
#if 0

bool Convert_PointCloud_To_PCL_Show_Double_In1Windows(iscas::PointCloud& pointcloud_ori, iscas::PointCloud& pointcloud_hdl)
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

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_data_ori, 255, 255,0);// yellow
	viewer.addPointCloud(pcl_data_ori, cloud_tr_color_h, "cloud_ori");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(pcl_data_hdl, 255, 0,0); // red
	viewer.addPointCloud(pcl_data_hdl, cloud_icp_color_h, "cloud_hdl");

	viewer.addCoordinateSystem(0.0);
	viewer.initCameraParameters();

    //viewer.setCameraPosition(0,0,2,0,2,0,0);  // -1 0 2 2 0 0 0
    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

    //viewer_pvs->spinOnce(100);  // 100ms
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
	return 0 ;
}

bool Convert_PointCloud_To_PCL_Show_Double_In2Windows(iscas::PointCloud& pointcloud_ori, iscas::PointCloud& pointcloud_hdl)
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

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    viewer.setCameraPosition(0,0,2,0,2,0,0);

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
	return 0 ;
}

bool Convert_PointCloud_To_PCL_Continue_Show(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer_pvs,  iscas::PointCloud& pointcloud_in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data( new pcl::PointCloud<pcl::PointXYZ>() );
    size_t point_size = pointcloud_in.width * pointcloud_in.height;
    if(pointcloud_data->points.size() != point_size)
    {
        pointcloud_data->points.resize(point_size);
    }
    printf("Show point size = %ld \n",point_size);
    for(size_t i=0;i<point_size;i++)
    {
        pointcloud_data->points[i]._PointXYZ::x = pointcloud_in.x_set[i];
        pointcloud_data->points[i]._PointXYZ::y = pointcloud_in.y_set[i];
        pointcloud_data->points[i]._PointXYZ::z = pointcloud_in.z_set[i];
		// std::cout<<"ori : i:"<< i << ", x:"<<pcl_data_ori->points[i]._PointXYZ::x <<
        //     ",y:"<<pcl_data_ori->points[i]._PointXYZ::y<<",z:"<<pcl_data_ori->points[i]._PointXYZ::z<<std::endl;
    }
    pointcloud_data->width = pointcloud_in.width;
    pointcloud_data->height = pointcloud_in.height;
    pointcloud_data->header.stamp = pointcloud_in.time_stamp;
    pointcloud_data->header.frame_id = "camera_link_ori";

    viewer_pvs->setBackgroundColor (0, 0, 0); // config backend color:dark
    viewer_pvs->addCoordinateSystem (1.0);    // show coordinate xyz
    viewer_pvs->initCameraParameters ();
    viewer_pvs->initCameraParameters ();

    //viewer_pvs->setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6],CameraPose[7],CameraPose[8],CameraPose[9]); // -1,0,0,0,0,1,0);
    viewer_pvs->setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);
        // 0 0 2 0 2 0 0 ## z轴看不到，y 朝上，X水平
		// 前面3位，相当于是相机的位置，只有一位有值，则相机在轴线上，看不到对应的轴，如果2位有值，则相机在这两个轴线的中间夹角45度，有值的两个方向成90度夹角面对屏幕
		// 中间3位，视角方向，哪个有值，则看向哪个轴（面），有值的那个轴（面）朝上

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (pointcloud_data, 0, 255, 0);

    viewer_pvs->removeAllPointClouds();  // 移除当前所有点云
    viewer_pvs->addPointCloud<pcl::PointXYZ>(pointcloud_data, single_color, "sample cloud");
    viewer_pvs->updatePointCloud(pointcloud_data, "sample cloud");

    viewer_pvs->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.0001, "sample cloud");// modify show size
	viewer_pvs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "sample_cloud");	//设置点云显示的颜色，rgb 在 [0,1] 范围
	viewer_pvs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "sample_cloud");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】

    viewer_pvs->spinOnce(100);  // 100ms
    // while (!viewer_pvs->wasStopped ())
    // {
    //     viewer_pvs->spinOnce();
    // }
    return 0;
}
bool Readxyz_ShowDouble(iscas::PointCloud& pnt_cloud_origin, iscas::PointCloud& pnt_cloud)
{
    s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 } ;
    static int index = 35 ;
    readSunnyOripcl("/home/qli/Desktop/xyz", "ori", index  ,read_pcl);
    covertToPointCloud(&tmp_data[0], read_pcl , pnt_cloud_origin);

    memset(read_pcl,0,sizeof(s_PointData)*PCL_FRAME_LENGTH);
    readSunnyOripcl("/home/qli/Desktop/xyz", "rgb", index  ,read_pcl);
    covertToPointCloud(&tmp_data[1] , read_pcl , pnt_cloud);
    index ++ ;

	if(Convert_PointCloud_To_PCL_Show_Double_In2Windows(pnt_cloud_origin, pnt_cloud))
	{
		std::cout << "Convert_PointCloud_To_PCL_Show_Double_In2Windows failed . " << std::endl;
		return -1;
	}
    return 0;
}

bool Readxyz_And_ContinueShow(iscas::PointCloud& pnt_cloud_origin)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_pvs(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 } ;
    static int index = 25 ;
    for(int i = index ; i < 70 ; i++)
    {
        readSunnyOripcl("/home/qli/Desktop/xyz", "rgb", i  ,read_pcl);
        covertToPointCloud(&tmp_data[0], read_pcl , pnt_cloud_origin);

        if(Convert_PointCloud_To_PCL_Continue_Show(viewer_pvs, pnt_cloud_origin))
        {
            std::cout << "Convert_PointCloud_To_PCL_Continue_Show failed . " << std::endl;
            return -1;
        }
    }
    return 0;
}
#endif
