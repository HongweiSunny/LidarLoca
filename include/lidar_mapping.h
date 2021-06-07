// #pragma once
#ifndef LIDAR_MAPPING_H_
#define LIDAR_MAPPING_H_

#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include "lidarFactor.hpp"
#include "common.h"
#include "tic_toc.h"
using namespace std;

class LidarMapping
{
public:
    int frameCount = 0;

    double timeLaserCloudCornerLast = 0;
    double timeLaserCloudSurfLast = 0;
    double timeLaserCloudFullRes = 0;
    double timeLaserOdometry = 0;

    int laserCloudCenWidth;
    int laserCloudCenHeight;
    int laserCloudCenDepth;
    int laserCloudWidth;
    int laserCloudHeight;
    int laserCloudDepth;

    int laserCloudNum;

    int laserCloudValidInd[125];
    int laserCloudSurroundInd[125];

    // input: from odom
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;

    // ouput: all visualble cube points
    pcl::PointCloud<PointType>::Ptr laserCloudSurround;

    // surround points in map to build tree
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;

    //input & output: points in one frame. local --> global
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

    // points in every cube  可以用vector
    // pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[4851];
    // pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[4581];
    vector<pcl::PointCloud<PointType>::Ptr > laserCloudCornerArray;
    vector< pcl::PointCloud<PointType>::Ptr > laserCloudSurfArray;

    //kd-tree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    double parameters[7];
    Eigen::Map<Eigen::Quaterniond> q_w_curr;
    Eigen::Map<Eigen::Vector3d> t_w_curr;

    // wmap_T_odom * odom_T_curr = wmap_T_curr;
    // transformation between odom's world and map's world frame
    Eigen::Quaterniond q_wmap_wodom;
    Eigen::Vector3d t_wmap_wodom;

    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;

    // 队列
    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;

    //
    std::mutex mBuf;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel;

    // ros
    // pub
    ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

    nav_msgs::Path laserAfterMappedPath;

    ros::NodeHandle nh;

    // sub
    ros::Subscriber subLaserCloudCornerLast;

    ros::Subscriber subLaserCloudSurfLast;

    ros::Subscriber subLaserOdometry;

    ros::Subscriber subLaserCloudFullRes;

    float lineRes;
    float planeRes;

public:
    LidarMapping() : laserCloudCornerLast(new pcl::PointCloud<PointType>()), laserCloudSurfLast(new pcl::PointCloud<PointType>()),
                     laserCloudSurround(new pcl::PointCloud<PointType>()), laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
                     laserCloudSurfFromMap(new pcl::PointCloud<PointType>()), laserCloudFullRes(new pcl::PointCloud<PointType>()),
                     kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()), kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>()),
                     q_wmap_wodom(1, 0, 0, 0), t_wmap_wodom(0, 0, 0), q_wodom_curr(1, 0, 0, 0), t_wodom_curr(0, 0, 0),
                     parameters{0, 0, 0, 1, 0, 0, 0}, q_w_curr(parameters), t_w_curr(parameters + 4)
    {
        laserCloudCenWidth = 10;
        laserCloudCenHeight = 10;
        laserCloudCenDepth = 5;
        laserCloudWidth = 21;
        laserCloudHeight = 21;
        laserCloudDepth = 11;

        laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

        lineRes = 0;
        planeRes = 0;
        nh.param<float>("mapping_line_resolution", lineRes, 0.4);
        nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
        printf("\n line resolution  %f plane resolution %f \n", lineRes, planeRes);
        downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
        downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

        // ros
        // sub
        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_corner_last", 100, &LidarMapping::laserCloudCornerLastHandler, this);

        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_surf_last", 100, &LidarMapping::laserCloudSurfLastHandler, this);

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, &LidarMapping::laserOdometryHandler, this);

        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_3", 100, &LidarMapping::laserCloudFullResHandler, this);

        // pub
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

        pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

        pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_cloud_registered", 100);

        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

        pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

        pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

//
#ifdef DEBUG
        cout << "设置数组前" << endl;
#endif
        laserCloudCornerArray.resize(laserCloudNum);
        laserCloudSurfArray.resize(laserCloudNum);
        for (int i = 0; i < laserCloudNum; i++)
        {
            laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
            laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
            // cout << i << endl;
        }
#ifdef DEBUG
        cout << "构造完成" << endl;
#endif
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2);

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2);

    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2);

    //receive odomtry
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

    // 友元函数  在外部定义
    // friend void process_map_test(LidarMapping &LM);  // 可以不用设置友元 除非访问private成员

    //
    // set initial guess
    void transformAssociateToMap();

    void transformUpdate();

    // 用Mapping的位姿w_curr，将Lidar坐标系下的点变换到world坐标系下
    void pointAssociateToMap(PointType const *const pi, PointType *const po);

    void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

    void process();

    // 同步检测
    bool sync_to_corner_detect(); 

    // 点云同步检测
    bool sync_cloud_time_detect();

    void take_cloud_from_buf();

}; // end of class

// 测试线程函数
void process_map_test(LidarMapping &LM);

// 建图的线程函数
void process_map_func(LidarMapping &LM);




#endif

