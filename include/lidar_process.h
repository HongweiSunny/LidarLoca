#pragma once

#ifndef LIDAR_PROCESS_H_
#define LIDAR_PROCESS_H_

#include <iostream>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>
#include "common.h"
#include "tic_toc.h"
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace pcl;

/*class LidarProcess
{
private:
    pcl::PointCloud<PointType>::Ptr cloudIn_ptr;
    pcl::PointCloud<PointType>::Ptr cloudOut_ptr;

public:
    LidarProcess(){};
    ~LidarProcess(){};

    void qujibian_func(const pcl::PointCloud<PointType> &, const pcl::PointCloud<PointType> &); // 去畸变的函数
    // void dist_filter_func(pcl::PointCloud&, pcl::PointCloud&); // 距离滤波
    // void voxel_filter_func(pcl::PointCloud&, pcl::PointCloud&); // 体素滤波器
};*/

class LidarConfig // 可以写成单例模式
{
public:
    uint16_t numScan = 16;
    double scanPeriod = 0.1;
    uint16_t get_num_scan() { return numScan; };
    double get_scan_period() { return scanPeriod; };

public:
    LidarConfig()
    { /*numScan = 16;*/
    }
    LidarConfig(uint16_t n) { numScan = n; }
};
/*
class LidarConfigSingle   // 可以写成单例模式
{
    public:
        const uint16_t numScan = 16;
        uint16_t get_num_scan() { return numScan; };
        static boost::shared_ptr<LidarConfigSingle> lidarConfigSingle_ptr = nullptr;
        static createLidarConfigSingle()
        {
            if(lidarConfigSingle_ptr == nullptr)
            {
                lidarConfigSingle_ptr = new LidarConfigSingle();
            }
        }
    private:
        LidarConfigSingle() { numScan = 16; };
};*/

/** Scan Registration configuration parameters. */
/*
// class RegistrationParams
// {
// public:
//     RegistrationParams(const float &scanPeriod_ = 0.1,
//                        const int &imuHistorySize_ = 200,
//                        const int &nFeatureRegions_ = 6,
//                        const int &curvatureRegion_ = 5,
//                        const int &maxCornerSharp_ = 2,
//                        const int &maxSurfaceFlat_ = 4,
//                        const float &lessFlatFilterSize_ = 0.2,
//                        const float &surfaceCurvatureThreshold_ = 0.1){};

//     /** The time per scan. */
//     float scanPeriod;

//     /** The size of the IMU history state buffer. */
//     int imuHistorySize;

//     /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
//     int nFeatureRegions;

//     /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
//     int curvatureRegion;

//     /** The maximum number of sharp corner points per feature region. */
//     int maxCornerSharp;

//     /** The maximum number of less sharp corner points per feature region. */
//     int maxCornerLessSharp;

//     /** The maximum number of flat surface points per feature region. */
//     int maxSurfaceFlat;

//     /** The voxel size used for down sizing the remaining less flat surface points. */
//     float lessFlatFilterSize;

//     /** The curvature threshold below / above a point is considered a flat / corner point. */
//     float surfaceCurvatureThreshold;
// };
//

class LidarPreProcess
{
public:

    string topicSub_str;
    string topicPub_str;

    ros::NodeHandle nh;
    ros::Subscriber lidarSub;

    TicToc tcal;

    pcl::PointCloud<PointType>::Ptr cloudByScans_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriPtr = nullptr; // (  new pcl::PointCloud<pcl::PointXYZ>()  ); // 加括号  才能构造出来一个对象
                                                               //  pcl::PointCloud<PointType>::Ptr cloudPreProcessPtr =  nullptr;// 加括号  才能构造出来一个对象

    pcl::PointCloud<pcl::PointXYZ> cloudOri;
    //pcl::PointCloud<PointType> cloudByScans; // 按照线的顺序组成的新的点云
    pcl::PointCloud<PointType> cloudPreProcess;

    // pcl::PointCloud<PointType> cloudLine;    // 线特征
    // pcl::PointCloud<PointType> cloudSurf;    // 面特征

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    pcl::VoxelGrid<PointType> downSizeFilter; // 降采样器

    std::vector<pcl::PointCloud<PointType>> cloudScans; // 存放不同线上的点

    LidarConfig lidarConf;

    std::vector<int> scanStartInd; // 把所有线的点都放在一个vector中
    std::vector<int> scanEndInd;

    float cloudCurvature[400000];    // 存放曲率
    int cloudSortInd[400000];        // 存放选中的点
    int cloudNeighborPicked[400000]; // 存放领域是否被选中
    int cloudLabel[400000];          // 存放点的类型标志

    ros::Publisher pubLaserCloud;        // 点云发布器
    ros::Publisher pubCornerPointsSharp; // 尖锐的角特征发布器
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat; // 平坦的面特征的发布器
    ros::Publisher pubSurfPointsLessFlat;
    ros::Publisher pubRemovePoints;          // 被移除的点云发布器
    std::vector<ros::Publisher> pubEachScan; // 发布器的容器

public:
    LidarPreProcess() : lidarConf(16)
    {
        topicSub_str = "/rslidar_points"; // /rslidar_points

        lidarSub = nh.subscribe(topicSub_str, 10, &LidarPreProcess::lidar_callback_func, this);

        scanStartInd.resize(lidarConf.get_num_scan());
        scanEndInd.resize(lidarConf.get_num_scan());
        cloudByScans_ptr = boost::make_shared<pcl::PointCloud<PointType>>();

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_by_scans", 100);
        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_sharp", 100);
        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_less_sharp", 100);
        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_flat", 100);
        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_less_flat", 100);
        pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/process/cloud_remove_points", 100);
    }

    ~LidarPreProcess(){};

    // 去除最近点的一个函数模板
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, double thres)
    {
        if (&cloud_in != &cloud_out) // 比较两个对象的地址
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;

            if (isnan(cloud_in.points[i].x) || isnan(cloud_in.points[i].y) || isnan(cloud_in.points[i].z))
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j); // 本质上是个 vector 重新调整size
        }
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    // 删除离群点
    void dist_filter(pcl::PointCloud<PointXYZ> &cloudIn, pcl::PointCloud<PointXYZI> &cloudOut);

    // 提取特征
    void extract_line(pcl::PointCloud<PointXYZ> &cloudIn);

    void extract_feature();

    void lidar_callback_func(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg_in);

    void publish_point_cloud(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg_in);


};

#endif