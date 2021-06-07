#pragma once

#ifndef LidarQujibianS_H_
#define LidarQujibianS_H_

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
class LidarQujibian
{
private:
    pcl::PointCloud<PointType>::Ptr cloudIn_ptr;
    pcl::PointCloud<PointType>::Ptr cloudOut_ptr;

public:
    LidarQujibian(){};
    ~LidarQujibian(){};

    void qujibian_func(const pcl::PointCloud<PointType>&, const pcl::PointCloud<PointType>&); // 去畸变的函数
    // void dist_filter_func(pcl::PointCloud&, pcl::PointCloud&); // 距离滤波
    // void voxel_filter_func(pcl::PointCloud&, pcl::PointCloud&); // 体素滤波器

};

class LidarConfig   // 可以写成单例模式
{
    public:
         uint16_t numScan = 16;
         double scanPeriod = 0.1;
        uint16_t get_num_scan() { return numScan; };
        double get_scan_period() { return scanPeriod; };

    public:
        LidarConfig() { /*numScan = 16;*/ }
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
class RegistrationParams
{
public:
    RegistrationParams(const float& scanPeriod_ = 0.1,
      const int& imuHistorySize_ = 200,
      const int& nFeatureRegions_ = 6,
      const int& curvatureRegion_ = 5,
      const int& maxCornerSharp_ = 2,
      const int& maxSurfaceFlat_ = 4,
      const float& lessFlatFilterSize_ = 0.2,
      const float& surfaceCurvatureThreshold_ = 0.1);

    /** The time per scan. */
    float scanPeriod;

    /** The size of the IMU history state buffer. */
    int imuHistorySize;

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
    int nFeatureRegions;

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
    int curvatureRegion;

    /** The maximum number of sharp corner points per feature region. */
    int maxCornerSharp;

    /** The maximum number of less sharp corner points per feature region. */
    int maxCornerLessSharp;

    /** The maximum number of flat surface points per feature region. */
    int maxSurfaceFlat;

    /** The voxel size used for down sizing the remaining less flat surface points. */
    float lessFlatFilterSize;

    /** The curvature threshold below / above a point is considered a flat / corner point. */
    float surfaceCurvatureThreshold;
  };


class LidarPreProcess
{
    public:
        // LidarQujibian lidarQJ;

        string topicSub_str;
        string topicPub_str;

        ros::NodeHandle nh;
        ros::Subscriber lidarSub;
        ros::Publisher lidarPub;

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

        float cloudCurvature[400000]; // 存放曲率
        int cloudSortInd[400000];  // 存放选中的点
        int cloudNeighborPicked[400000]; // 存放领域是否被选中
        int cloudLabel[400000];  // 存放点的类型标志

        ros::Publisher pubLaserCloud; // 点云发布器
        ros::Publisher pubCornerPointsSharp; // 尖锐的角特征发布器
        ros::Publisher pubCornerPointsLessSharp;
        ros::Publisher pubSurfPointsFlat; // 平坦的面特征的发布器
        ros::Publisher pubSurfPointsLessFlat;
        ros::Publisher pubRemovePoints;  // 被移除的点云发布器
        std::vector<ros::Publisher> pubEachScan; // 发布器的容器

    public:
        LidarPreProcess():lidarConf(16)
        {
            topicSub_str = "/rslidar_points";  // /rslidar_points
            topicPub_str = "/processed_points";
            lidarSub = nh.subscribe(topicSub_str, 10, &LidarPreProcess::lidar_callback_func, this);
            lidarPub = nh.advertise<sensor_msgs::PointCloud2>(topicPub_str, 10);
            scanStartInd.resize(lidarConf.get_num_scan());
            scanEndInd.resize(lidarConf.get_num_scan());
            cloudByScans_ptr = boost::make_shared<pcl::PointCloud<PointType>>();

            pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_by_scans", 100);
            pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/cloud_sharp", 100);
            pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("cloud_less_sharp", 100);
            pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/cloud_flat", 100);
            pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/cloud_less_flat", 100);
            pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/cloud_remove_points", 100);

        }

        LidarPreProcess(const string& topicSub_, const string& topicPub_):topicSub_str(topicSub_), topicPub_str(topicPub_),lidarConf(16)
        { // 可以用委托构造函数  简化一下
            lidarSub = nh.subscribe(topicSub_str, 10, &LidarPreProcess::lidar_callback_func, this);
            lidarPub = nh.advertise<sensor_msgs::PointCloud2>(topicPub_str, 10);
            scanStartInd.resize(lidarConf.get_num_scan());
            scanEndInd.resize(lidarConf.get_num_scan());
            cloudByScans_ptr = boost::make_shared<pcl::PointCloud<PointType>>();
            pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_by_scans", 100);
            pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/cloud_sharp", 100);
            pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("cloud_less_sharp", 100);
            pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/cloud_flat", 100);
            pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/cloud_less_flat", 100);
            pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/cloud_remove_points", 100);
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
                
                if(isnan( cloud_in.points[i].x ) || isnan( cloud_in.points[i].y) || isnan( cloud_in.points[i].z ) )
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
        void dist_filter(pcl::PointCloud<PointXYZ>& cloudIn, pcl::PointCloud<PointXYZI>& cloudOut)  
        {
            // cloudOut.points.reserve(cloudIn.points.size());
            for (auto &p : cloudIn.points)
            {
                double x = p.x;
                double y = p.y;
                double z = p.z;
                double dis = x * x + y * y + z * z;
                if (dis>=0.5*0.5 && dis<=60.0*60.0)
                {
                    // cloudOut->points.emplace_back(x,y,z);
                    pcl::PointXYZI pp;
                    pp.x = x;
                    pp.y = y;
                    pp.z = z;
                    pp.intensity = 0.0f;
                    cloudOut.points.emplace_back(pp);
                }
            }
        }

        // 提取特征
        void extract_line(pcl::PointCloud<PointXYZ>& cloudIn)
        {
            // 点分成不同的线  借鉴了laboshinl的写法
            cloudScans.resize(lidarConf.get_num_scan());
            // clear all scanline points  
            std::for_each(cloudScans.begin(), cloudScans.end(), [](pcl::PointCloud<PointType>  &v) {v.points.clear(); }); // 引用才能清除
            cout << "清除vector后" << endl;
            std::for_each(cloudScans.begin(), cloudScans.end(), [](pcl::PointCloud<PointType> v)
                          { cout << v.points.size() << " "; });
            cout << endl;

            // vector<int> vvv(10, 0);
            // for_each(vvv.begin(), vvv.end(), [](int it) { cout << it << endl; });
            // 处理每个激光点
            uint cloudSize = cloudIn.size();
            double startOri = -atan2(cloudIn.points[0].y, cloudIn.points[0].x); // 为什么是负值
            double endOri = -atan2(cloudIn.points[cloudSize - 1].y,  cloudIn.points[cloudSize - 1].x) + 2 * M_PI;
            cout << "startOri: " << startOri << endl;
            cout << "endOri: " << endOri << endl;
            if (endOri - startOri > 3 * M_PI)
            {
                endOri -= 2 * M_PI;
            }
            else if (endOri - startOri < M_PI)
            {
                endOri += 2 * M_PI;
            }
            for (int i = 0; i < cloudSize; i++)
            {
                pcl::PointXYZI p;
                p.x = cloudIn.points[i].x;
                p.y = cloudIn.points[i].y;
                p.z = cloudIn[i].z;  // 也可以不通过points 直接索引

                // skip NaN and INF valued points
                if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y) ||  !pcl_isfinite(p.z))
                 { continue; }

                // skip zero valued points
                if (p.x * p.x + p.y * p.y + p.z * p.z < 0.3) { continue;}

                // calculate vertical point angle and scan ID
                double angle = std::atan(p.z/ std::sqrt(p.x * p.x + p.y * p.y))* 180 / M_PI;
                int scanID = 0;
                scanID = int((angle + 15) / 2 + 0.5);                      // 速腾的角度和velodyne的角度不同 ==TODO==
                if (scanID > (lidarConf.get_num_scan() - 1) || scanID < 0) // 为什么要减去1
                {
                    continue;
                 }
                double ori = -atan2(p.y, p.x);
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)   // 没明白这里的逻辑
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }

                double relTime = (ori - startOri) / (endOri - startOri); // 这个角度代表的是在旋转一圈的时候 该点对应的百分比
                if(isnan(relTime))
                {
                    cout << "NaN: "
                         << "endOri" <<endOri
                         << "  startOri: " << startOri << endl;
                }
                p.intensity = scanID + lidarConf.get_scan_period() * relTime;        // SCAN_ID是第几条线束
                // cout << "intensity" << p.intensity << endl;
                cloudScans[scanID].push_back(p);
            }
        }

        void extract_feature()
        {
            // 按各个线组成新的有序的点云
            // pcl::PointCloud<PointType>::Ptr cloudByScans_ptr;
            if(cloudByScans_ptr->points.size()!=0)
                cloudByScans_ptr->points.clear();
            cout << "提取特征前: " << cloudByScans_ptr->size() << endl;

            int count = 0;
            for (int i = 0; i < lidarConf.get_num_scan(); i++) // 这个循环是在做什么?
            { 
                scanStartInd[i] = cloudByScans_ptr->size() + 5;
                *cloudByScans_ptr += cloudScans[i];
                count += cloudScans[i].size();
                scanEndInd[i] = cloudByScans_ptr->size() - 6;
            }
            cout << "集合各个线后的点云大小：" << count << endl;
            //
            cout << " " << cloudByScans_ptr->size() << endl;
            // 计算曲率
            int cloudSizeByScans = cloudByScans_ptr->size();
            for (int i = 5; i < cloudSizeByScans - 5; i++)
            { 
                double diffX = cloudByScans_ptr->points[i - 5].x + cloudByScans_ptr->points[i - 4].x + cloudByScans_ptr->points[i - 3].x + cloudByScans_ptr->points[i - 2].x + cloudByScans_ptr->points[i - 1].x - 10 * cloudByScans_ptr->points[i].x + cloudByScans_ptr->points[i + 1].x + cloudByScans_ptr->points[i + 2].x + cloudByScans_ptr->points[i + 3].x + cloudByScans_ptr->points[i + 4].x + cloudByScans_ptr->points[i + 5].x;
                double diffY = cloudByScans_ptr->points[i - 5].y + cloudByScans_ptr->points[i - 4].y + cloudByScans_ptr->points[i - 3].y + cloudByScans_ptr->points[i - 2].y + cloudByScans_ptr->points[i - 1].y - 10 * cloudByScans_ptr->points[i].y + cloudByScans_ptr->points[i + 1].y + cloudByScans_ptr->points[i + 2].y + cloudByScans_ptr->points[i + 3].y + cloudByScans_ptr->points[i + 4].y + cloudByScans_ptr->points[i + 5].y;
                double diffZ = cloudByScans_ptr->points[i - 5].z + cloudByScans_ptr->points[i - 4].z + cloudByScans_ptr->points[i - 3].z + cloudByScans_ptr->points[i - 2].z + cloudByScans_ptr->points[i - 1].z - 10 * cloudByScans_ptr->points[i].z + cloudByScans_ptr->points[i + 1].z + cloudByScans_ptr->points[i + 2].z + cloudByScans_ptr->points[i + 3].z + cloudByScans_ptr->points[i + 4].z + cloudByScans_ptr->points[i + 5].z;

                cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 这个数组太大了
                cloudSortInd[i] = i; 
                cloudNeighborPicked[i] = 0;
                cloudLabel[i] = 0;
            }

            // 下面选取特征点
            // 把之前提取的特征点都清空
            cornerPointsLessSharp.clear();
            cornerPointsSharp.clear();
            surfPointsFlat.clear();
            surfPointsLessFlat.clear();
            for (int i = 0; i < lidarConf.get_num_scan(); i++)
            {
                if( scanEndInd[i] - scanStartInd[i] < 6)
                    continue;

                pcl::PointCloud<PointType>::Ptr surfPointsLessFlatEachScan_ptr(new pcl::PointCloud<PointType>);
                for (int j = 0; j < 6; j++)  // 分成6部分
                {
                    int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
                    int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                    std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, [this](int i, int j)
                              { return this->cloudCurvature[i] < this->cloudCurvature[j]; }); // 对该部分进行曲率的排序 排好的索引放在cloudSortInd中

                    // 选取sharp的点
                    int largestPickedNum = 0;
                    for (int k = ep; k >= sp; k--)
                    {
                        int ind = cloudSortInd[k]; 
                        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                        {
                            largestPickedNum++;
                            if (largestPickedNum <= 2) 
                            {                        
                                cloudLabel[ind] = 2;
                                cornerPointsSharp.push_back(cloudByScans_ptr->points[ind]); // 每段最多放两个点进去
                                cornerPointsLessSharp.push_back(cloudByScans_ptr->points[ind]);
                            }
                            else if (largestPickedNum <= 20) 
                            {                        
                                cloudLabel[ind] = 1; 
                                cornerPointsLessSharp.push_back(cloudByScans_ptr->points[ind]); // 每段最多放20个点进去
                            }
                            else
                            {
                                break;
                            }

                            cloudNeighborPicked[ind] = 1; 
                            for (int l = 1; l <= 5; l++)
                            {
                                float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l - 1].x;
                                float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l - 1].y;
                                float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l - 1].z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) // 如果周围的点的曲率大于一定值
                                {
                                    break;
                                }
                                cloudNeighborPicked[ind + l] = 1; // 周围的点的曲率没有达到阈值时，标记为1
                            }
                            for (int l = -1; l >= -5; l--)
                            {
                                float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l + 1].x;
                                float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l + 1].y;
                                float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l + 1].z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                {
                                    break;
                                }
                                cloudNeighborPicked[ind + l] = 1;
                            }
                        }
                    }

                    // 选取flat的点
                    int smallestPickedNum = 0;
                    for (int k = sp; k <= ep; k++)
                    {
                        int ind = cloudSortInd[k];
                        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                        {
                            cloudLabel[ind] = -1; 
                            surfPointsFlat.push_back(cloudByScans_ptr->points[ind]);  
                            smallestPickedNum++;
                            if (smallestPickedNum >= 4)  // 选取四个点
                            { 
                                break;
                            }

                            cloudNeighborPicked[ind] = 1;
                            for (int l = 1; l <= 5; l++)
                            { 
                                float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l - 1].x;
                                float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l - 1].y;
                                float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l - 1].z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)   // TODO 为什么这里也是大于0.05？ 选择平面点的时候不应该是太小的标记吗？
                                {
                                    break;
                                }

                                cloudNeighborPicked[ind + l] = 1;
                            }
                            for (int l = -1; l >= -5; l--)
                            {
                                float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l + 1].x;
                                float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l + 1].y;
                                float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l + 1].z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                {
                                    break;
                                }

                                cloudNeighborPicked[ind + l] = 1;
                            }
                        }
                    }

                    // 得到剩余的大部分的平面点
                    for (int k = sp; k <= ep; k++)
                    {
                        if (cloudLabel[k] <= 0)   // 所以大部分都是less flat的点 ，其他三种只有20 4 2 这种级别的数量
                        {
                            surfPointsLessFlatEachScan_ptr->push_back(cloudByScans_ptr->points[k]);
                        }
                    }
                }

                pcl::PointCloud<PointType> surfPointsLessFlatScanDS; // 降采样后的点
                downSizeFilter.setInputCloud(surfPointsLessFlatEachScan_ptr); // 每条线中的lessflat的点被降采样，放入临时对象中
                downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
                downSizeFilter.filter(surfPointsLessFlatScanDS);
                surfPointsLessFlat += surfPointsLessFlatScanDS;  // 降采样后的点放到less flat的点集中
            }

        }
        void lidar_callback_func(   const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg_in)
        {
            
            #ifdef DEBUG
                        cout << "get lidar data!" << endl;
            #endif
            // 在这里进行点云的去畸变处理
            // 调用
                // pcl::PointCloud<pcl::PointXYZ> cloudOri;   // 用XYZI会报错
            
            cloudOri.clear();
            cloudPreProcess.clear();
            tcal.tic();

            pcl::fromROSMsg(*pointCloudMsg_in, cloudOri); //该函数的第二个形参只能是pointcloud形式
            std::vector<int> indices;
            // pcl::removeNaNFromPointCloud(cloudOri, cloudOri, indices);    // 先去掉NaN的点 is_dense的情况下不会滤去NaN的点...
            // cout << "是否dense: " << cloudOri.is_dense
            //      << endl;
            // dist_filter(cloudOri, cloudPreProcess);
            cout << "原始点云的大小： " << cloudOri.size() << endl;
            removeClosedPointCloud(cloudOri, cloudOri, MINIMUM_RANGE);  // 去掉比较近的点  仍然把点存在ori中
            cout << "remove后点云的大小： " << cloudOri.size() << endl;
            cout << tcal.toc() << endl;

            // 发布距离处理后的点云
            // tcal.tic();
            // sensor_msgs::PointCloud2 processedMsg;
            // pcl::toROSMsg(   cloudPreProcess,  processedMsg);
            // processedMsg.header.stamp = pointCloudMsg_in->header.stamp;
            // processedMsg.header.frame_id = "/rslidar";
            // lidarPub.publish(processedMsg);
            // cout << "publish time: " << tcal.toc() << endl;  // 发布不占多少时间  1ms都不到

            // // 点云扫描分层
            tcal.tic();
            extract_line(cloudOri);
            cout << tcal.toc() << " -扫描分层时间" << endl;

            tcal.tic();
            extract_feature();
            cout << "提取特征的时间： " << tcal.toc() << endl;

            // 发布特征点
            sensor_msgs::PointCloud2 laserCloudOutMsg;    // 按scan排好的点云
            pcl::toROSMsg(*cloudByScans_ptr, laserCloudOutMsg);
            laserCloudOutMsg.header.stamp = pointCloudMsg_in->header.stamp;
            laserCloudOutMsg.header.frame_id = "rslidar";
            pubLaserCloud.publish(laserCloudOutMsg);

            sensor_msgs::PointCloud2 cornerPointsSharpMsg;   // sharp的角点
            pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
            cornerPointsSharpMsg.header.stamp = pointCloudMsg_in->header.stamp;
            cornerPointsSharpMsg.header.frame_id = "rslidar";
            pubCornerPointsSharp.publish(cornerPointsSharpMsg);

            sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
            pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
            cornerPointsLessSharpMsg.header.stamp = pointCloudMsg_in->header.stamp;
            cornerPointsLessSharpMsg.header.frame_id = "rslidar";
            pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

            sensor_msgs::PointCloud2 surfPointsFlat2;
            pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
            surfPointsFlat2.header.stamp = pointCloudMsg_in->header.stamp;
            surfPointsFlat2.header.frame_id = "rslidar";
            pubSurfPointsFlat.publish(surfPointsFlat2);

            sensor_msgs::PointCloud2 surfPointsLessFlat2;
            pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
            surfPointsLessFlat2.header.stamp = pointCloudMsg_in->header.stamp;
            surfPointsLessFlat2.header.frame_id = "rslidar";
            pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

            // // pub each scam
            if(1)
            {
                     for(int i = 0; i < lidarConf.get_num_scan(); i++)
                    {
                        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
                        pubEachScan.push_back(tmp);
                    }
                for(int i = 0; i< lidarConf.get_num_scan(); i++)
                {
                    sensor_msgs::PointCloud2 scanMsg;
                    pcl::toROSMsg(cloudScans[i], scanMsg);
                    scanMsg.header.stamp = pointCloudMsg_in->header.stamp;
                    scanMsg.header.frame_id = "/rslidar";
                    pubEachScan[i].publish(scanMsg);
                }
            }

        }

        


};

#endif