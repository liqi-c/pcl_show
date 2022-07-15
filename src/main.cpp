#include "pcl_show.hpp"

typedef enum {
    REGULAR_FILE,
    DIR_FILE
}e_file_type;
int main(int argc, char *argv[])
{
	int i;
	struct stat buf;
	char *ptr;
    int type[2]={};
    if(0 != argc){
        printf("argc = %d \n",argc);
        for (i = 1; i < argc; i++) 
        {
            if (lstat(argv[i], &buf) < 0) 
            {
                perror("lstat");
                continue;
            }
            if(S_ISREG(buf.st_mode))
            {    
                ptr = "regular file";
                type[i-1]= REGULAR_FILE;
            }
            else if (S_ISDIR(buf.st_mode))	
            {    
                ptr = "directory file";
                type[i-1]= DIR_FILE;
            }
            else
                ptr = "** unknown mode **";
            
            printf("[%d] %s: %s.[%d]\n",i, argv[i],ptr,type[i-1]);
        }
    }
    switch(argc)
    {
        case 2:
            if(REGULAR_FILE == type[0])
            {
                Read_PCD_and_Show_Single(argv[1]);
            }
            else  //dir 
            {
                Read_PCD_and_Show_Continue_Single(argv[1]);
            }
            break;
        case 3:
            if(REGULAR_FILE == type[0] && REGULAR_FILE == type[1] )
            {
                Read_PCD_and_Show_Double(argv[1],argv[2]);
            }
            else if(DIR_FILE == type[0] && DIR_FILE == type[1] )
            {
                Read_PCD_and_Show_Continue_Double(argv[1],argv[2]);
            }
            break;
        default :
            printf("Error!\n");
    }
    // Read_PCD_and_Show_Single("/home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd");
    // Read_PCD_and_Show_Double("/home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd","/home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd");

    // Read_PCD_and_Show_Continue_Double("/home/liq/CODE/ori_pcl","/home/liq/CODE/hdl_pcl");
    // Read_PCD_and_Show_Continue_Single("/home/liq/CODE/ori_pcl");
 
    return 0;
}
