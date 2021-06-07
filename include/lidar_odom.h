#pragma once

#include <iostream>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "lidarFactor.hpp"
#include  "common.h"
#include "tic_toc.h"

using namespace std;

class LidarOdom
{
    public:
        ros::NodeHandle nh;
        ros::Rate rate;

        int corner_correspondence = 0;
        int  plane_correspondence = 0;
        

        // 控制建图频率
        int skipFrameNum = 2;  // 5Hz
        int frameCount = 0;
        bool systemInited = false;

        // 迭代优化次数
        int MAX_ITERATION = 2;

        double timeCornerPointsSharp = 0;
        double timeCornerPointsLessSharp = 0;
        double timeSurfPointsFlat = 0;
        double timeSurfPointsLessFlat = 0;
        double timeLaserCloudFullRes = 0;
        double SCAN_PERIOD = 0.1;
        double DISTANCE_SQ_THRESHOLD = 25;
        double NEARBY_SCAN = 2.5;  // 常量表达式 在编译的时候就会直接替代
        bool DISTORTION = true;

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

        pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

        int laserCloudCornerLastNum = 0;
        int laserCloudSurfLastNum = 0;

        // q_curr_last(x, y, z, w), t_curr_last
        double para_q[4];
        double para_t[3];
        
        Eigen::Map<Eigen::Quaterniond> q_last_curr;
        Eigen::Map<Eigen::Vector3d> t_last_curr;
        // Eigen::Quaterniond q_last_curr;
        // Eigen::Vector3d t_last_curr;

        // // Transformation from current frame to world frame
        Eigen::Quaterniond q_w_curr;
        Eigen::Vector3d t_w_curr;

        // 点云缓存队列
        std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
        
        // 互斥量
        std::mutex mBuf; 

        // 订阅器
        ros::Subscriber subCornerPointsSharp;
        void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr&);

        ros::Subscriber subCornerPointsLessSharp;
        void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &);

        ros::Subscriber subSurfPointsFlat;
        void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &);

        ros::Subscriber subSurfPointsLessFlat;
        void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &);

        ros::Subscriber subLaserCloudFullRes;
        void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &);

        // 发布器
        ros::Publisher pubLaserCloudCornerLast;

        ros::Publisher pubLaserCloudSurfLast;

        ros::Publisher pubLaserCloudFullRes;

        ros::Publisher pubLaserOdometry;

        ros::Publisher pubLaserPath;

        // 路径
        nav_msgs::Path laserPath;



        ~LidarOdom(){};

public:
        LidarOdom() : rate(10), para_q{0., 0., 0., 1.}, para_t{0., 0., 0.}, q_last_curr(para_q), t_last_curr(para_t), q_w_curr(1, 0, 0, 0), t_w_curr(0, 0, 0)
        {
                kdtreeCornerLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI> >( );
                kdtreeSurfLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

                cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
                cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
                surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
                surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType> >();

                laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
                laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType> >();
                laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();

                //
                subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_sharp", 100, &LidarOdom::laserCloudSharpHandler, this);
                subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_less_sharp", 100, &LidarOdom::laserCloudLessSharpHandler, this);
                subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_flat", 100, &LidarOdom::laserCloudFlatHandler, this);
                subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_less_flat", 100, &LidarOdom::laserCloudLessFlatHandler, this);
                subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_by_scans", 100, &LidarOdom::laserCloudFullResHandler, this);
                //
                pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/cloud_corner_last", 100);
                pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/cloud_surf_last", 100);
                pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_3", 100);
                pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
                pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

                //t_last_curr << 0,0,0;
                //t_w_curr << 0, 0, 0;
        };

        // 成员函数
        void spin_once();

        void calculate_pose_nlp();

        void pub_result();

        void after_process();

        void TransformToStart(PointType const *const pi, PointType *const po);

        void TransformToEnd(PointType const *const pi, PointType *const po);
};
