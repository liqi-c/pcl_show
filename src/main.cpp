#include "isca_tof_camera.h"
#include "pcl_show.hpp"
#include <stdio.h>
// /#include <io.h>
#include <string>

extern int CameraPose[10];
extern s_xyzdata tmp_data[2];

#include <dirent.h>
#include <unistd.h>
// bool Find_File_From_Dir(char * basePath, char * file_name[])
// {
//     DIR *dir;
//     struct dirent *ptr;
//     char base[1000];
 
//     if ((dir=opendir(basePath)) == NULL)
//     {
//         perror("Open dir error...");
//         exit(1);
//     }
//     while ((ptr=readdir(dir)) != NULL)
//     {
//         if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
//             continue;
//         else if(ptr->d_type == 8)    ///file
//         {
//             printf("file_name:%s/%s\n",basePath,ptr->d_name);
//             //file_name = ptr->d_name ;
//             sprintf(file_name,"%s/%s",basePath,ptr->d_name);
//         }    
//         else if(ptr->d_type == 10)    ///link file
//             printf("link_file:%s/%s\n",basePath,ptr->d_name);
//         else if(ptr->d_type == 4)    ///dir
//         {
//             printf("Dir : %s/%s.\n",basePath,ptr->d_name);
//             // memset(base,'\0',sizeof(base));
//             // strcpy(base,basePath);
//             // strcat(base,"/");
//             // strcat(base,ptr->d_name);
//             // readFileList(base);
//         }
//     }
//     closedir(dir);

// }
#define MAX_FILENAME_LENGTH 100 

int main(int argc, char *argv[])
{
    std::cout << "main func has been called!" << std::endl;
    //constructor
    iscas::handle_t h = iscas::tof_camera_create();
    //function call

    if(0 != argc){
        printf("[%d] -%s",argc,argv[0]);
        for(int i = 1 ; i < argc ; i++ )
        {
            printf("-%s-",argv[i]);
            CameraPose[i-1] = atoi(argv[i]);
        }
        printf("\n");
    }

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

    DIR *dir;
    struct dirent *ptr;
    char base[1000];
	char file_name[MAX_FILENAME_LENGTH]={};
    char * basePath = "/home/liq/CODE/ori_pcl";
    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            sprintf((char *)file_name,"%s/%s",basePath,ptr->d_name);
            printf("file_name:%s\n",file_name);
        }
#if  0
        s_PointData read_pcl[PCL_FRAME_LENGTH] = { 0 };
        static int index = 25 ;  
        readSunnyXyz("/home/liq/CODE/ori_16.xyz" , read_pcl);
        Convert_SunnyXyz_To_IscasPCL(&tmp_data[0], read_pcl , pnt_cloud_origin);
#else
        Read_Pcd(file_name , pointcloud_ori);
        if(PCL_Show_Continue_One(viewer_pvs, pointcloud_ori ))
        {
 			std::cout << "PCL_Show_Double_In_2_Windows failed . " << std::endl;
			return -1;           
        }
        continue ;
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
    closedir(dir);

    iscas::tof_camera_destroy(h);
    return 0;
}
