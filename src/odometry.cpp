//
// Created by znfs on 2021/11/29.
//

#include "tic_toc.h"
#include "load_param.h"

using Eigen::Vector3d;

// 上一帧plane,ground点，
pcl::PointCloud<PointType> laserCloudIn_plane_last;

// 暂时保存上述点的容器，先进先出
std::queue<sensor_msgs::PointCloud2ConstPtr> planeBuf;
std::mutex mBuf;

// 局部里程计， 整体里程计， 路径， 当前全部点， 地图点
ros::Publisher pubLaser_Local_Odometry;
ros::Publisher pubLaserPath;
ros::Publisher pubLaser_Odom_cloud;
ros::Publisher pubgroundPath;

// 定义路径，用于保存帧的位置，发布与pubLaserPath
nav_msgs::Path laserPath;
nav_msgs::Path groundPath;

// 到世界坐标系下
Eigen::Quaterniond q_w_curr(1,0,0,0);
Eigen::Vector3d t_w_curr(0,0,0);

// 当前帧到上一帧
double para_q[4] = {0,0,0,1};
double para_t[3] = {0,0,0};

// 四元数q， 当前帧到上一帧
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
// 当前帧到上帧位移量t，配合四元数累加在一起就是当前帧到最开始帧也就是世界坐标系
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

constexpr double DISTANCE_SQ_THRESHOLD = 9;
constexpr double NEARBY_SCAN = 2.5;

int map_msg_count = 0;

// 构建代价函数，residual为残差
struct CURVE_PLANE_COST
{
    CURVE_PLANE_COST(Eigen::Vector3d _curr_point_a_, Eigen::Vector3d _last_point_b_,
                     Eigen::Vector3d _last_point_c_, Eigen::Vector3d _last_point_d_):
            curr_point_a_(_curr_point_a_),last_point_b_(_last_point_b_),
            last_point_c_(_last_point_c_),last_point_d_(_last_point_d_)
    {
        plane_norm = (last_point_d_ - last_point_b_).cross(last_point_c_ - last_point_b_);
        plane_norm.normalize();
    }

    template<typename T>
    bool operator()(const T* q, const T* t, T* residual)const{
        Eigen::Matrix<T, 3, 1> p_a_curr{T(curr_point_a_.x()), T(curr_point_a_.y()), T(curr_point_a_.z())};
        Eigen::Matrix<T, 3, 1> p_b_last{T(last_point_b_.x()), T(last_point_b_.y()), T(last_point_b_.z())};
        Eigen::Matrix<T, 3, 1> last_plane_norm(T(plane_norm.x()), T(plane_norm.y()), T(plane_norm.z()));
        Eigen::Quaternion<T> rot_q{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> rot_t{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> p_a_last;
        p_a_last = rot_q * p_a_curr + rot_t;
        residual[0] = (p_a_last - p_b_last).dot(last_plane_norm);
        return true;
    }

    const Eigen::Vector3d curr_point_a_,last_point_b_,last_point_c_,last_point_d_;
    Eigen::Vector3d plane_norm;
};

void TransformToLast(PointType const *const pi, PointType *const po){
    Vector3d point(pi->x, pi->y, pi->z);
    Vector3d un_point;

    un_point = q_last_curr * point + t_last_curr;

    //输出一下
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

void cloud_plane_Callback(const sensor_msgs::PointCloud2Ptr &plane_cloud){
    mBuf.unlock();
    planeBuf.push(plane_cloud);
    mBuf.unlock();
}

void tf_Callback(const tf::tfMessage msg_tf){
    double msg_X = msg_tf.transforms[0].transform.translation.x;
    double msg_Y = msg_tf.transforms[0].transform.translation.y;
    double msg_Z = msg_tf.transforms[0].transform.translation.z;

    //输出轨迹
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "map";
    laserOdometry.child_frame_id = "os_lidar";
    laserOdometry.header.stamp = msg_tf.transforms[0].header.stamp;
    laserOdometry.pose.pose.orientation.x = 0;
    laserOdometry.pose.pose.orientation.y = 0;
    laserOdometry.pose.pose.orientation.z = 0;
    laserOdometry.pose.pose.orientation.w = 1;
    laserOdometry.pose.pose.position.x = msg_X;
    laserOdometry.pose.pose.position.y = msg_Y;
    laserOdometry.pose.pose.position.z = msg_Z;

    geometry_msgs::PoseStamped laserPose;
    laserPose.header = laserOdometry.header;
    laserPose.pose = laserOdometry.pose.pose;
    groundPath.header.stamp = laserOdometry.header.stamp;
    groundPath.poses.push_back(laserPose);
    groundPath.header.frame_id = "map";
    pubgroundPath.publish(groundPath);
}

void odom_thread(){
    while(1){
        sleep(0.1);
        if(!planeBuf.empty()){
            double timeplane = planeBuf.front()->header.stamp.toSec();
            pcl::PointCloud<PointType> laserCloudIn_plane;

            mBuf.lock();
            pcl::fromROSMsg(*planeBuf.front(), laserCloudIn_plane);
            planeBuf.pop();
            mBuf.unlock();

            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(para_q, 4, q_parameterization);
            problem.AddParameterBlock(para_t, 3);

            pcl::KdTreeFLANN<PointType> kdtreePlaneLast;
            // 上一帧的点数和这一帧的点数，上一帧点数足够在进行位姿估计
            int laserCloudIn_plane_last_num = laserCloudIn_plane_last.points.size();
            int laserCloudIn_plane_num = laserCloudIn_plane.points.size();

            if(laserCloudIn_plane_last_num < 10)
                laserCloudIn_plane_last = laserCloudIn_plane;
            else{
                kdtreePlaneLast.setInputCloud(laserCloudIn_plane_last.makeShared());
                for(int optize_num=0; optize_num<=1; optize_num++){
                    // 对于这一帧中的每个点，用a表示，进行寻找上一帧最近点b，并根据b点寻找同线或上一线上的c以及下一线上的d进行点面距离估计
                    // 并加入优化函数中
                    for(int i=0; i<laserCloudIn_plane_num; i++){
                        PointType pointseed;
                        std::vector<int> pointSearchInd;
                        std::vector<float> pointSearchSqDis;
                        TransformToLast(&laserCloudIn_plane.points[i], &pointseed);
                        kdtreePlaneLast.nearestKSearch(pointseed, 1, pointSearchInd, pointSearchSqDis);

                        // b,c,d的id， b点是kd输求出的; closestPoint_scanID是b点的线号
                        int closestPointInd = pointSearchInd[0];
                        int minPointInd2=-1;
                        int minPointInd3=-1;
                        int closestPoint_scanID=laserCloudIn_plane_last.points[pointSearchInd[0]].b;

                        // 如果ab距离小于3, 那么继续进行， 否则不优化，直接下一帧
                        if(pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){
                            // 向b点的线号往上寻找c点
                            for(int j=closestPointInd+2; (j<laserCloudIn_plane_last_num)&&(minPointInd2==-1);j++){
                                if(laserCloudIn_plane_last.points[j].b > closestPoint_scanID+NEARBY_SCAN)
                                    continue;
                                else{
                                    double distance_a_c=(laserCloudIn_plane_last.points[j].x - pointseed.x)
                                                       *(laserCloudIn_plane_last.points[j].x - pointseed.x)
                                                       +(laserCloudIn_plane_last.points[j].y - pointseed.y)
                                                       *(laserCloudIn_plane_last.points[j].y - pointseed.y)
                                                       +(laserCloudIn_plane_last.points[j].z - pointseed.z)
                                                       *(laserCloudIn_plane_last.points[j].z - pointseed.z);
                                    if(distance_a_c > DISTANCE_SQ_THRESHOLD)
                                        continue;
                                    else
                                        minPointInd2 = j;
                                }
                            }

                            // 向b点的线号往下寻找d点（不可以同线号）
                            // 使用if取并集来实现不共线
                            for(int j=closestPointInd-1;(j>0)&&(minPointInd3==-1);j--){
                                if((laserCloudIn_plane_last.points[j].b<closestPoint_scanID-NEARBY_SCAN)
                                || (laserCloudIn_plane_last.points[j].b==closestPoint_scanID))
                                    continue;
                                else{
                                    double distance_a_d=(laserCloudIn_plane_last.points[j].x-pointseed.x)
                                                       *(laserCloudIn_plane_last.points[j].x-pointseed.x)
                                                       +(laserCloudIn_plane_last.points[j].y-pointseed.y)
                                                       *(laserCloudIn_plane_last.points[j].y-pointseed.y)
                                                       +(laserCloudIn_plane_last.points[j].z-pointseed.z)
                                                       *(laserCloudIn_plane_last.points[j].z-pointseed.z);
                                    if(distance_a_d>DISTANCE_SQ_THRESHOLD)
                                        continue;
                                    else
                                        minPointInd3=j;
                                }
                            }

                            if(minPointInd2==-1||minPointInd3==-1)
                                continue;
                            else{
                                Vector3d last_point_a(laserCloudIn_plane.points[i].x,
                                                      laserCloudIn_plane.points[i].y,
                                                      laserCloudIn_plane.points[i].z);
                                Eigen::Vector3d curr_point_b(laserCloudIn_plane_last.points[closestPointInd].x,
                                                             laserCloudIn_plane_last.points[closestPointInd].y,
                                                             laserCloudIn_plane_last.points[closestPointInd].z);
                                Eigen::Vector3d curr_point_c(laserCloudIn_plane_last.points[minPointInd2].x,
                                                             laserCloudIn_plane_last.points[minPointInd2].y,
                                                             laserCloudIn_plane_last.points[minPointInd2].z);
                                Eigen::Vector3d curr_point_d(laserCloudIn_plane_last.points[minPointInd3].x,
                                                             laserCloudIn_plane_last.points[minPointInd3].y,
                                                             laserCloudIn_plane_last.points[minPointInd3].z);
                                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_PLANE_COST,1,4,3>
                                                        (new CURVE_PLANE_COST(last_point_a,curr_point_b,
                                                         curr_point_c,curr_point_d)),loss_function,para_q,para_t);
                            }
                        }
                    }

                }
            }

            // 对所有a点遍历过后进行优化
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 5;

            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // 优化后，当前帧变成前一帧，位姿累积，并且根据位姿变换当前帧所有点到世界坐标系，放入地图中
            laserCloudIn_plane_last = laserCloudIn_plane;
            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;

            pcl::PointCloud<PointType>::Ptr laserCloud_odom_curr(new pcl::PointCloud<PointType>());

            for(int i=0;i<laserCloudIn_plane_num;i++)
            {
                PointType point_in_map;
                TransformToLast(&laserCloudIn_plane.points[i], &point_in_map);
                laserCloud_odom_curr->push_back(point_in_map);
            }

            sensor_msgs::PointCloud2 laserCloudMapMsg;
            pcl::toROSMsg(*laserCloud_odom_curr, laserCloudMapMsg);
            laserCloudMapMsg.header.stamp = ros::Time().fromSec(timeplane);
            laserCloudMapMsg.header.frame_id = "map";
            pubLaser_Odom_cloud.publish(laserCloudMapMsg);

            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "os_lidar";
            laserOdometry.header.stamp = ros::Time().fromSec(timeplane);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaser_Local_Odometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "map";
            pubLaserPath.publish(laserPath);
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Subscriber cloud_plane = nh.subscribe("/point_cloud_plane", 100, cloud_plane_Callback);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 100, tf_Callback);

    pubLaser_Odom_cloud = nh.advertise<sensor_msgs::PointCloud2>("/os_cloud_map", 100);
    pubLaser_Local_Odometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    boost::thread odom_thread_(odom_thread);

    ros::spin();
    return 0;
}