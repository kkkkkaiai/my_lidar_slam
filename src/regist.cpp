//
// Created by znfs on 2021/11/26.
//

#include "tic_toc.h"
#include "load_param.h"


//输出全部点，平面点，地面点
ros::Publisher pubLaserCloudall;
ros::Publisher pubLaserCloudplane;
ros::Publisher pubLaserCloudground;

// 全部点，包含一些其他信息
pcl::PointCloud<PointType>::Ptr laserCloudall(new pcl::PointCloud<PointType>());
// 平面点
pcl::PointCloud<PointType>::Ptr laserCloudplane(new pcl::PointCloud<PointType>());
// 地面点
pcl::PointCloud<PointType>::Ptr laserCloudground(new pcl::PointCloud<PointType>());

helti::LoadParam sensor_param = helti::LoadParam::HeltiParam();

//点的序号，初始为1-laserCloudall.size()顺序排列，后续进行了排序操作
int cloudSortInd[400000];
//曲率存放
double cloudcurv[400000];

std::vector<float> verticalAngles = {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15};
int verticleAnglesMaxIndex = verticalAngles.size() - 1;
float lowerBound = -45;
float upperBound = 45;
int beam = 64;
float factor = (beam-1)/(upperBound - lowerBound);

//用于排序，输入i和j，获取对应点的曲率对比，在后续使用中，会将cloudSortInd的某一区间指代的点曲率从小到大排序
bool comp (int i,int j) { return (cloudcurv[i]<cloudcurv[j]); }

//int find_verticle_index(float& angle){
//    return lround(((angle*180 / M_PI) - lowerBound) * factor);
//}

int find_verticle_index(float& angle){
    return lround(((angle*180 / M_PI) - lowerBound) * factor);
}

int find_verticle_index(std::vector<float>& verticalAngles, float angle, float threshold = 0.01){
    int index = -1;
    float minvalue = MAXFLOAT;

    for(size_t i = 0; i < verticalAngles.size(); i++){
        auto temp = std::abs(verticalAngles[i] - angle);
        if(temp < threshold && temp < minvalue){
            index = i;
            minvalue = temp;
        }
    }
    return index;
}

void cloud_callback(const sensor_msgs::PointCloud2 input_cloud){
    pcl::PointCloud<pcl::PointXYZ> source;
    pcl::fromROSMsg(input_cloud, source);
    pcl::PointCloud<pcl::PointXYZ> CloudIn;
//    pcl::transformPointCloud(source, CloudIn, sensor_param.os_lidar_->transform_.matrix());

    Quaterniond quat(1, 0, 0, 0);
    Eigen::Vector4d quaternion_;
    Isometry3d transform_;
    Vector3d translation_;
    quaternion_ << 1, 0, 0, 0;
    transform_ = Eigen::Isometry3d::Identity();
    transform_.rotate(quat.toRotationMatrix());
    translation_ << 0, 0, 0;
    transform_.pretranslate(translation_);
    pcl::transformPointCloud(source, CloudIn, transform_.matrix());

    auto cloudSize = CloudIn.points.size();
    PointType point;

    std::vector<pcl::PointCloud<PointType>> laserCloudScans(beam);
    laserCloudall->clear();
    laserCloudplane->clear();
    laserCloudground->clear();

    for(size_t i=0; i < cloudSize; i++){
        point.x = CloudIn.points[i].x;
        point.y = CloudIn.points[i].y;
        point.z = CloudIn.points[i].z;
        point.r = 1; point.g = 1;
//        std::cout << "x " << source.points[i].x << " " << CloudIn.points[i].x << std::endl;
//        std::cout << source.points[i].y << " " << CloudIn.points[i].y << std::endl;
//        std::cout << source.points[i].z << " " << CloudIn.points[i].z << std::endl;
        float angle = atan(point.z / sqrt(point.x*point.x + point.y*point.y));
        int scanID = find_verticle_index(angle);
//        if(scanID < 2){
//            std::cout << point.x << " " << point.y << " " << point.z << std::endl;
//            std::cout << "angle " << angle << " cal "<< angle*180/M_PI << " scan ID  " << scanID << std::endl;
//        }
        point.b = scanID;
        if(scanID < 0 || scanID >= beam)
            continue;
        laserCloudScans[scanID].push_back(point);
    }

    for(int i=0; i< int(beam*0.5); i++){
        for(int j=0; j<int(laserCloudScans[i].size()) && j<int(laserCloudScans[i+1].size()); j++){
            float diffX,diffY,diffZ,angle;
            diffX = laserCloudScans[i+1].points[j].x - laserCloudScans[i].points[j].x;
            diffY = laserCloudScans[i+1].points[j].y - laserCloudScans[i].points[j].y;
            diffZ = laserCloudScans[i+1].points[j].z - laserCloudScans[i].points[j].z;
            angle = atan(diffZ / sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
            if(abs(angle) < 2)
            {
                laserCloudground->push_back(laserCloudScans[i+1].points[j]);
                laserCloudground->push_back(laserCloudScans[i].points[j]);
            }
        }
    }
//    std::cout << laserCloudground->points.size() << std::endl;
    // compare curvature
    long ind_count = 0;
    // log the end position of everyscan
    int cloudScanEndInd[beam+1];
    cloudScanEndInd[0]=0;

    // calculate curvature
    for(int i=0;i < beam;i++)
    {
        for(int j=5; j<int(laserCloudScans[i].size())-5; j++)
        {
            float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
            float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
            float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;
            cloudcurv[ind_count] = (diffX * diffX + diffY * diffY + diffZ * diffZ);
            laserCloudall->points.push_back(laserCloudScans[i].points[j]);
            ind_count++;
            cloudSortInd[ind_count] = ind_count;
        }
        cloudScanEndInd[i+1] = laserCloudall->points.size();
    }

    int laser_num = laserCloudall->points.size();
    for(int i=0; i<beam; i++){
        int start_num = cloudScanEndInd[i];
        int end_num = cloudScanEndInd[i+1];
        for(int j=0; j < 6; j++){
            int start_num_temp = start_num + ((end_num-start_num)/6)*j;
            int end_num_temp = start_num + ((end_num-start_num)/6)*(j+1);

            std::sort(cloudSortInd+start_num_temp, cloudSortInd+end_num_temp, comp);

            int plane_num = 0;
            for(int k=start_num_temp; k<end_num_temp&&plane_num<5; k++){
                long ind = cloudSortInd[k];
                //选取可选点以及曲率小的点
                if(laserCloudall->points[ind].r == 1 && cloudcurv[ind]<0.1)
                {
                    plane_num++;
                    laserCloudall->points[ind].g = plane_num;
                    laserCloudplane->push_back(laserCloudall->points[ind]);
                    //临近点变成不可选
                    for(int m=1; ind+m<laser_num&&m<=5;m++){
                        laserCloudall->points[ind+m].r=2;
                    }
                    for(int m=1; ind-m>0&&m<=5;m++){
                        laserCloudall->points[ind-m].r=2;
                    }
                }

            }
        }
    }



    sensor_msgs::PointCloud2 laserCloudplaneMsg;
    pcl::toROSMsg(*laserCloudplane, laserCloudplaneMsg);
    laserCloudplaneMsg.header.stamp = input_cloud.header.stamp;
    laserCloudplaneMsg.header.frame_id = "os_lidar";
    pubLaserCloudplane.publish(laserCloudplaneMsg);


    sensor_msgs::PointCloud2 laserCloudgroundMsg;
    pcl::toROSMsg(*laserCloudground, laserCloudgroundMsg);
    laserCloudgroundMsg.header.stamp = input_cloud.header.stamp;
    laserCloudgroundMsg.header.frame_id = "os_lidar";
    pubLaserCloudground.publish(laserCloudgroundMsg);

    sensor_msgs::PointCloud2 laserCloudallMsg;
    pcl::toROSMsg(*laserCloudall, laserCloudallMsg);
    laserCloudallMsg.header.stamp = input_cloud.header.stamp;
    laserCloudallMsg.header.frame_id = "os_lidar";
    pubLaserCloudall.publish(laserCloudallMsg);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "regist");
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub = nh.subscribe("/os_cloud_node/points", 100, cloud_callback);
    pubLaserCloudground = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_ground", 100);
    pubLaserCloudall = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_all", 100);
    pubLaserCloudplane = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_plane", 100);

    ros::spin();
}