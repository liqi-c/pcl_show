// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
// Author : qli@openailab.com

#include "pcl_show.hpp"
#include "pcl_remove_groud.hpp"
s_xyzdata tmp_data[2] ;

/*
CameraPose 参数说明：
 (pos_x,pos_y,pos_z)为相机光心，
 (view_x, view_y, view_z)为光轴上的一点，他们的差向量决定了光轴方向，
 (up_x, up_y, up_z)确定相机的正上方

	## 前面3位，相当于是相机的位置，只有一位有值，则相机在轴线上，看不到对应的轴，如果2位有值，则相机在这两个轴线的中间夹角45度，有值的两个方向成90度夹角面对屏幕
	## 中间3位，视角方向，哪个有值，则看向哪个轴（面），有值的那个轴（面）朝上
   viewer_pvs.addCoordinateSystem (0.1);    // 参数越大，坐标轴越宽

配置示例：
    0 0 2 0 2 0 0 ## z轴看不到，y 朝上，X水平
    0 0 2 2 0 0 0 ## z轴看不到，x 朝上，y水平
    -2 0 2 2 0 0 0 ## x,z轴夹角面向屏幕，y水平
    -2 0 1 2 0 0 0 ## x,z轴夹角面向屏幕，y水平

 */
int CameraPose[7]={-2,0,1,2,0,0,0};


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
bool PCL_Show_Signle(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_data, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgbz(pcl_data, "z"); // 配置沿着Z轴的颜色变化
	viewer.addPointCloud<pcl::PointXYZ>(pcl_data, rgbz, "cloud_ori");

	viewer.addCoordinateSystem(0.1);

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ori");// modify show size

	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "cloud_ori");//红色，设置点云显示的颜色，rgb 在 [0,1] 范围
	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud_ori");//黑色，设置点云显示的颜色，rgb 在 [0,1] 范围

	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud_ori");		//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】
	// viewer.initCameraParameters();

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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_data_ori, 255, 255, 0);// yellow
	viewer.addPointCloud(pcl_data_ori, cloud_tr_color_h, "cloud_ori");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(pcl_data_hdl, 255, 0,0); // red
	viewer.addPointCloud(pcl_data_hdl, cloud_icp_color_h, "cloud_hdl");

	viewer.addCoordinateSystem(0.0);
	viewer.initCameraParameters();

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
	return 0 ;
}
bool PCL_Show_Double_In_2_Windows(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_hdl)
{
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
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2", v2);
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

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgbz(pointcloud_data, "z"); // 配置沿着Z轴的颜色变化
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (pointcloud_data, 0, 255, 0);
    if(1 != init_flag) // add once .
    {
        viewer_pvs.addPointCloud<pcl::PointXYZ>(pointcloud_data, rgbz, "pointcloud_data");
    }

    viewer_pvs.updatePointCloud(pointcloud_data, rgbz, "pointcloud_data");

    viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pointcloud_data");// modify show size
	//viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.7, "pointcloud_data");	//设置点云显示的颜色，rgb 在 [0,1] 范围
	//viewer_pvs.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "pointcloud_data");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】

    viewer_pvs.spinOnce(1000);  // 100ms
    init_flag = 1;
    return 0;
}
bool PCL_Show_Continue_Double(pcl::visualization::PCLVisualizer viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    static bool init_flag = true;
    viewer.setBackgroundColor (0, 0, 0); // config backend color:dark
    viewer.addCoordinateSystem (0.1);    // show coordinate xyz

    viewer.setCameraPosition(CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5],CameraPose[6]);

	static int v1(0);  //创建左窗口显式cloud1
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb1(cloud1, "z");

    if(true == init_flag)
	{
        viewer.createViewPort(0, 0, 0.5, 1.0, v1);  //左右窗口大小划分，1:1
	    viewer.setBackgroundColor(0, 0, 0, v1);
    	// viewer.addText("Cloud1", 2, 2, "Cloud1", v1);  //窗口下的标题
        viewer.addPointCloud<pcl::PointXYZ>(cloud1, rgb1, "cloud1", v1);
    }
    viewer.updatePointCloud(cloud1, rgb1, "cloud1");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1", v1);

	static int v2(1);  //创建右窗口显示cloud2
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb2(cloud2, "z");

    if(true == init_flag)
	{
        viewer.createViewPort(0.5, 0, 1.0, 1.0, v2);  //左右窗口大小划分，1:1
       // viewer.setBackgroundColor(0.75, 0.75, 0.75, v2);
        viewer.setBackgroundColor(1, 1, 1, v2); // black
	    viewer.addPointCloud<pcl::PointXYZ>(cloud2, rgb2, "cloud2", v2);
    }
    viewer.updatePointCloud(cloud2, rgb2, "cloud2");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud2",v2);

    viewer.spinOnce(1000);  // 100ms
 //   viewer.removeAllPointClouds();
 //   viewer.removeAllTexts();
    init_flag = false;

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
bool Read_PCD_and_Show_Signle_Alg(const char * life_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

    Read_Pcd(life_path , pointcloud_ori);

    ProcessPointCloud(pointcloud_ori, pointcloud_hdl);

    if(PCL_Show_Double_In_2_Windows( pointcloud_ori, pointcloud_hdl))
    {
        std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
        return -1;
    }
    return 0;
}
bool Read_PCD_and_Show_Signle_AlgSeg(const char * life_path)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());
/*Step 1 : 读取pcd文件 */
    Read_Pcd(life_path , pointcloud_ori);
/*Step 2 : 去除地面 */
    ProcessPointCloud(pointcloud_ori, pointcloud_hdl);
/*Step 3 : 点云分割 */
    Pcl_Segmentation_EC(pointcloud_hdl);
/*Step 4 : 显示处理结果和原始点云 */
    if(PCL_Show_Double_In_2_Windows( pointcloud_ori, pointcloud_hdl))
    {
        std::cout << "PCL_Show_Continue_Double failed . " << std::endl;
        return -1;
    }
    return 0;
}
bool Read_PCD_and_Show_Continue_Alg(const char * life_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

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

        ProcessPointCloud(pointcloud_ori, pointcloud_hdl);

        if(o_ptr != NULL)
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
bool Read_PCD_and_Show_Continue_AlgSeg(const char * life_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ori(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_hdl(new pcl::PointCloud<pcl::PointXYZ>());

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

        struct timeval tv;
        gettimeofday(&tv, NULL);
        double start_time = tv.tv_sec + tv.tv_usec / 1e6, current_time;

        ProcessPointCloud(pointcloud_ori, pointcloud_hdl);

        gettimeofday(&tv, NULL);
        current_time = tv.tv_sec + tv.tv_usec / 1e6;
        printf("ProcessPointCloud Process: %lf ms\n", (current_time - start_time)*1e3);

        Pcl_Segmentation_EC(pointcloud_hdl);
        // Pcl_Segmentation_RegGrow(pointcloud_hdl);
        if(o_ptr != NULL)
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