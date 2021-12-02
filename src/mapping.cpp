//
// Created by znfs on 2021/11/30.
//

#include "tic_toc.h"
#include "load_param.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
using Eigen::Map;
using Eigen::Quaterniond;
using Eigen::Vector3d;

//输出路径和地图
ros::Publisher pubLaserPath;
ros::Publisher pubLaserGtsamPath;
ros::Publisher pubLaserMap;

//存储每一帧的时间，用于gtsam优化后输出
std::vector<ros::Time> all_time;

//缓存plane点和odom算出的位姿，
//这里的plane是经过odometry.cpp里的q_last_curr，t_last_curr变换过的，也就是这些点在上一帧的坐标系，这是为了和地图匹配,地图目前只有第一帧到当前帧的点，
//所以为了匹配，需要像里程计里当前帧变前一帧再匹配一样，把当前帧变到上一帧坐标系，再由ros节点接收，再通过世界到前一帧变换关系,转换到世界坐标系下
//这里的odom是里程计计算出的从里程计原点到当前的位姿变换，也就是odometry.cpp里的q_w_curr，t_w_curr
std::queue<sensor_msgs::PointCloud2ConstPtr> cloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> planeBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

double parameters[7] = {0,0,0,1,0,0,0};
// 世界坐标系下某个点的四元数和位移
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters+4);
// 暂存上一帧的四元数和位移
Eigen::Quaterniond q_w_last(1,0,0,0);
Eigen::Vector3d t_w_last(0,0,0);

Eigen::Quaterniond q_wmap_wodom(1,0,0,0);
Eigen::Vector3d t_wmap_wodom(0,0,0);



///< transformation between map's world and current lidar frame
gtsam::Pose3 T_w_curr(gtsam::Rot3(Eigen::Matrix3d::Identity()), gtsam::Point3(0,0,0));
///< transformation between odom's world and map's world frame
gtsam::Pose3 T_wmap_wodom(gtsam::Rot3(Eigen::Matrix3d::Identity()), gtsam::Point3(0,0,0));
//里程计坐标系下某点的四元数和位移(对应rviz是绿线)
Eigen::Quaterniond  q_wodom_curr(1,0,0,0);
Eigen::Vector3d t_wodom_curr(0,0,0);
///< received laser odometry from current lidar frame to odom's world
gtsam::Pose3 T_wodom_curr(gtsam::Rot3(Eigen::Matrix3d::Identity()), gtsam::Point3(0,0,0));

//存放当前帧的全部点，plane点，位置临近帧的plane点构成的地图点云，我们使用plane点进行帧和局部地图（包括前200帧和历史位置临近的帧）的匹配，
//使用当前帧全部点和历史临近（也就是没有前200帧的位置临近帧）帧的全部点进行闭环检测
// 当前帧的全部点
pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
// 平面点
pcl::PointCloud<PointType>::Ptr laserCloudPlane(new pcl::PointCloud<PointType>());
// 包含所有平面点
pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>());

//用于icp的当前帧和历史帧(不能在前两百帧产生)的全部点
pcl::PointCloud<PointType>::Ptr laserCloud_now_out(new pcl::PointCloud<PointType>());
//icp的历史帧
pcl::PointCloud<PointType>::Ptr laserCloud_map_out(new pcl::PointCloud<PointType>());
//每二十帧降采样保存一次局部地图，会保存为pcd文件，在icp时根据位置提取出来，之所以要另存是因为保存在内存太大了
pcl::PointCloud<PointType>::Ptr laserCloud_local_map(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudBuf;
std::vector<gtsam::Pose3> laserCloudPose;

//由于plane点较少（每帧不到400点），我们保存全部plane点在内存里，即使10W点也就只有300MB左右，放内存可以承担
//laserCloudMap_Ind记录每帧plane点保存的终止时点数,暂定10万帧，比如laserCloudMap_Ind[200]=10086,
//就是第二百帧结束时有10086个点配合laserCloudMap_Ind[199]=10000，就可以精确定位第N帧的点提取出来。
//temp_laserCloudMap_Ind是指当前第几帧，会自加
//history_close_Ind指最近的历史帧，是第几帧，由位姿的KD树求出
long laserCloudMap_Ind[100000];
//当前为第几帧
long temp_laserCloudMap_Ind=0;
long history_close_Ind=0;

//这个是把每个地图帧的位姿保存在xyz里，之后用KD树找临近位姿，再根据位姿的ind从上面序号找临近帧的地图点进行匹配与闭环
pcl::PointCloud<PointType>::Ptr laserCloudMap_Pose(new pcl::PointCloud<PointType>());

pcl::VoxelGrid<PointType> downSizeFilterPoint;
pcl::VoxelGrid<PointType> downSizeFilterMap;
pcl::VoxelGrid<PointType> downSizeFilterICP;

//定义路径，用于保存帧的位置，发布于pubLaserPath
nav_msgs::Path laserPath;



//点面损失函数，输入的是当前帧的某点_point_o_，目标平面的中心点_point_a_，目标平面的法线_norn_，常规求ao向量在法向量上的投影
struct CURVE_PLANE_COST
{
    CURVE_PLANE_COST(Eigen::Vector3d _point_o_, Eigen::Vector3d _point_a_,Eigen::Vector3d _norn_):
            point_o_(_point_o_),point_a_(_point_a_),norn_(_norn_){}
    template <typename T>
    bool operator()(const T* q,const T* t,T* residual)const
    {
        Eigen::Matrix<T, 3, 1> p_o_curr{T(point_o_.x()), T(point_o_.y()), T(point_o_.z())};
        Eigen::Matrix<T, 3, 1> p_a_last{T(point_a_.x()), T(point_a_.y()), T(point_a_.z())};
        Eigen::Matrix<T, 3, 1> p_norm{T(norn_.x()), T(norn_.y()), T(norn_.z())};
        Eigen::Quaternion<T> rot_q{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> rot_t{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> p_o_last;
        p_o_last=rot_q * p_o_curr + rot_t;
        residual[0]=((p_o_last - p_a_last).dot(p_norm));
        return true;
    }
    const Eigen::Vector3d point_o_,point_a_,norn_;
};

//把某点转换为地图点
void TransformToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;

    un_point= q_w_curr * point + t_w_curr;

    //输出一下
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

//把某点转换为相对地图点
void TransformToMapZero(PointType const *const pi, PointType *const po,PointType *const center)
{
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;

    un_point= q_w_curr * point + t_w_curr;

    //输出一下
    po->x = un_point.x()-center->x;
    po->y = un_point.y()-center->y;
    po->z = un_point.z()-center->z;
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    gtsam::Point3 point_w = T_w_curr.transform_from(gtsam::Point3(point_curr.x(), point_curr.y(), point_curr.z()));

    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    //po->intensity = 1.0;
}

// set initial guess
void transformAssociateToMap()
{
    T_w_curr = T_wmap_wodom * T_wodom_curr;
}

void transformUpdate()
{
    T_wmap_wodom = T_w_curr * T_wodom_curr.inverse();
}

void cloud_Callback(const sensor_msgs::PointCloud2Ptr &cloud){
    mBuf.lock();
    cloudBuf.push(cloud);
    mBuf.unlock();
}

void cloud_plane_Callback(const sensor_msgs::PointCloud2Ptr &plane_cloud){
    mBuf.lock();
    planeBuf.push(plane_cloud);
    mBuf.unlock();
}

void local_odom_Callback(const nav_msgs::Odometry::ConstPtr &laser_odometry){
    mBuf.lock();
    odometryBuf.push(laser_odometry);
    mBuf.unlock();
}

void map_thread()
{
    gtsam::ISAM2Params gtsam_parameters;
    gtsam_parameters.relinearizeThreshold = 0.01;
    gtsam_parameters.relinearizeSkip = 1;
    ISAM2 isam(gtsam_parameters);

    NonlinearFactorGraph graph;
    Values initialEstimate;

    while(1) {


//        while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < planeBuf.front()->header.stamp.toSec())
//            odometryBuf.pop();
//
//        if (odometryBuf.empty()){
//            mBuf.unlock();
//            ROS_INFO("odometry buffer is empty");
//            break;
//        }
//
//        while(!cloudBuf.empty() && cloudBuf.front()->header.stamp.toSec() < planeBuf.front()->header.stamp.toSec())
//            cloudBuf.pop();
//
//        if(cloudBuf.empty()){
//            mBuf.unlock();
//            ROS_INFO("cloud buffer is empty");
//            break;
//        }
        sleep(0.1);
        while (!odometryBuf.empty() && !planeBuf.empty() && !cloudBuf.empty()) {
            mBuf.lock();
            //记录时间戳
            ROS_INFO("compare Timestamp");
            double timeLaserCloud = cloudBuf.front()->header.stamp.toSec();
            double timeLaserCloudPlane = planeBuf.front()->header.stamp.toSec();
            double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

            if (timeLaserCloudPlane != timeLaserOdometry || timeLaserCloud != timeLaserOdometry) {
                printf("unsync messeage! \n");
                mBuf.unlock();
                break;
            }

            //清空上次面特征点云，并接收新的
            laserCloudPlane->clear();
            pcl::fromROSMsg(*planeBuf.front(), *laserCloudPlane);
            planeBuf.pop();

            laserCloud->clear();
            pcl::fromROSMsg(*cloudBuf.front(), *laserCloud);
            cloudBuf.pop();

            //对点云进行下采样
            ROS_INFO("Downsample the cloud");
            downSizeFilterPoint.setInputCloud(laserCloud);
            downSizeFilterPoint.filter(*laserCloud);

            q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
            q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
            q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
            q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
            t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
            t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
            t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;

            T_wodom_curr = gtsam::Pose3(gtsam::Rot3(q_wodom_curr), gtsam::Point3(t_wodom_curr));
            odometryBuf.pop();
            mBuf.unlock();

            ///变换坐标系
            transformAssociateToMap();

            if (laserCloudPose.empty()) {
                noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
                        (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6).finished());
                graph.add(PriorFactor<Pose3>(laserCloudPose.size(), Pose3(gtsam::Rot3(q_wodom_curr),
                                                                          Point3(t_wodom_curr)), priorNoise));
                initialEstimate.insert(0, Pose3(gtsam::Rot3(q_wodom_curr), Point3(t_wodom_curr)));
            } else {
                gtsam::Pose3 lastPose = laserCloudPose.back();
                const gtsam::Pose3 &poseFrom = lastPose;
                gtsam::Pose3 poseTo = Pose3(gtsam::Rot3(q_wodom_curr), Point3(t_wodom_curr));
                noiseModel::Diagonal::shared_ptr odometryNoise =
                        noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6).finished());
                graph.add(
                        BetweenFactor<Pose3>(laserCloudPose.size() - 1, laserCloudPose.size(), poseFrom.between(poseTo),
                                             odometryNoise));
                initialEstimate.insert(laserCloudPose.size(), Pose3(gtsam::Rot3(q_wodom_curr), Point3(t_wodom_curr)));
            }

            ROS_INFO("isam update");
            isam.update(graph, initialEstimate);
            isam.update();

            graph.resize(0);
            initialEstimate.clear();

            ROS_INFO("give value to currentstate");
            Pose3 latestEstimate;
            Values isamCurrentEstimate;
            isamCurrentEstimate = isam.calculateEstimate();
            latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

            t_w_curr.x() = latestEstimate.translation().x();
            t_w_curr.y() = latestEstimate.translation().y();
            t_w_curr.z() = latestEstimate.translation().z();

            //把上一帧的位姿加入进Pose缓存中
            laserCloudPose.push_back(latestEstimate);

            size_t laserCloudNum = laserCloud->points.size();

            PointType pointOri, pointSel;
            for (int i = 0; i < laserCloudNum; i++) {
                pointOri = laserCloud->points[i];
                pointAssociateToMap(&pointOri, &pointSel);
                laserCloud->points[i].x = pointSel.x;
                laserCloud->points[i].y = pointSel.y;
                laserCloud->points[i].z = pointSel.z;
            }

            //转换后的点云累加起来
            *laserCloudMap += *laserCloud;
            //将点云存入进buffer中
            laserCloudBuf.push_back(laserCloud);

            if(laserCloudBuf.size() % 200 == 0){
                downSizeFilterMap.setInputCloud(laserCloudMap);
                downSizeFilterMap.filter(*laserCloudMap);
            }

            //publish laserPath
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "os_lidar";
            laserOdometry.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserOdometry.pose.pose.orientation.x = latestEstimate.rotation().toQuaternion().x();
            laserOdometry.pose.pose.orientation.y = latestEstimate.rotation().toQuaternion().y();
            laserOdometry.pose.pose.orientation.z = latestEstimate.rotation().toQuaternion().z();
            laserOdometry.pose.pose.orientation.w = latestEstimate.rotation().toQuaternion().w();
            laserOdometry.pose.pose.position.x = latestEstimate.translation().x();
            laserOdometry.pose.pose.position.y = latestEstimate.translation().y();
            laserOdometry.pose.pose.position.z = latestEstimate.translation().z();

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "map";
            pubLaserPath.publish(laserPath);

            //publish pointcloud
            sensor_msgs::PointCloud2 laserCloudMapMsg;
            pcl::toROSMsg(*laserCloudMap, laserCloudMapMsg);
            laserCloudMapMsg.header.stamp = ros::Time().fromSec(timeLaserCloudPlane);
            laserCloudMapMsg.header.frame_id = "map";
            pubLaserMap.publish(laserCloudMapMsg);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;

    downSizeFilterPoint.setLeafSize(1.,1.,1.);
    downSizeFilterMap.setLeafSize(0.5,0.5,0.5);
    downSizeFilterICP.setLeafSize(0.2,0.2,0.2);

    ros::Subscriber LaserCloudmap = nh.subscribe("/os_cloud_map", 100, cloud_plane_Callback);
    ros::Subscriber LaserLocalOdom = nh.subscribe("/laser_odom_to_init", 100, local_odom_Callback);
    ros::Subscriber cloud_sub = nh.subscribe("/point_cloud_all", 100, cloud_Callback);

    // 输出map后的轨迹与位姿
    pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_map_path", 100);
    pubLaserGtsamPath = nh.advertise<nav_msgs::Path>("/laser_gtsam_map_path", 100);
    pubLaserMap = nh.advertise<sensor_msgs::PointCloud2 >("/laser_map_cloud", 100);

    boost::thread server(map_thread);

    ros::spin();
    return 0;
}

