#include "lidar_process.h"

//void LidarProcess::qujibian_func(const pcl::PointCloud<PointType> &a, const pcl::PointCloud<PointType> &b)
// {
//     cout << "hh" << endl;
// }

// 删除离群点
void LidarPreProcess::dist_filter(pcl::PointCloud<PointXYZ> &cloudIn, pcl::PointCloud<PointXYZI> &cloudOut)
{
    // cloudOut.points.reserve(cloudIn.points.size());
    for (auto &p : cloudIn.points)
    {
        double x = p.x;
        double y = p.y;
        double z = p.z;
        double dis = x * x + y * y + z * z;
        if (dis >= 0.5 * 0.5 && dis <= 60.0 * 60.0)
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
void LidarPreProcess::extract_line(pcl::PointCloud<PointXYZ> &cloudIn)
{
    // 点分成不同的线  借鉴了laboshinl的写法
    cloudScans.resize(lidarConf.get_num_scan());
    // clear all scanline points
    std::for_each(cloudScans.begin(), cloudScans.end(), [](pcl::PointCloud<PointType> &v)
                  { v.points.clear(); }); // 引用才能清除
    cout << "清除vector后" << endl;
    std::for_each(cloudScans.begin(), cloudScans.end(), [](pcl::PointCloud<PointType> v)
                  { cout << v.points.size() << " "; });
    cout << endl;

    // vector<int> vvv(10, 0);
    // for_each(vvv.begin(), vvv.end(), [](int it) { cout << it << endl; });
    // 处理每个激光点
    uint cloudSize = cloudIn.size();
    double startOri = -atan2(cloudIn.points[0].y, cloudIn.points[0].x); // 为什么是负值
    double endOri = -atan2(cloudIn.points[cloudSize - 1].y, cloudIn.points[cloudSize - 1].x) + 2 * M_PI;
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
        p.z = cloudIn[i].z; // 也可以不通过points 直接索引
        // 这里没有对坐标进行处理 直接就是x前y左z上
        // skip NaN and INF valued points
        if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y) || !pcl_isfinite(p.z))
        {
            continue;
        }

        // skip zero valued points
        if (p.x * p.x + p.y * p.y + p.z * p.z < 0.3)
        {
            continue;
        }

        // calculate vertical point angle and scan ID
        double angle = std::atan(p.z / std::sqrt(p.x * p.x + p.y * p.y)) * 180 / M_PI;
        int scanID = 0;
        scanID = int((angle + 15) / 2 + 0.5);                      // 速腾的角度和velodyne的角度不同 ==TODO==
        if (scanID > (lidarConf.get_num_scan() - 1) || scanID < 0) // 
        {
            continue;
        }
        double ori = -atan2(p.y, p.x); // 激光雷达是逆时针旋转的
        ori += 2 * M_PI;
        if (ori < endOri - M_PI * 3 / 2) 
        {
            ori += 2 * M_PI;
        }
        else if (ori > endOri + M_PI / 2)
        {
            ori -= 2 * M_PI;
        }

        double relTime = (ori - startOri) / (endOri - startOri); // 这个角度代表的是在旋转一圈的时候 该点对应的百分比
        if (isnan(relTime))
        {
            cout << "NaN: "
                 << "endOri" << endOri
                 << "  startOri: " << startOri << endl;
            ROS_WARN("NaN realtime!\n");
            continue;
        }
        p.intensity = scanID + lidarConf.get_scan_period() * relTime; // SCAN_ID是第几条线束
        // cout << "intensity" << p.intensity << endl;
        cloudScans[scanID].push_back(p);
    }
}

void LidarPreProcess::extract_feature()
{
    // 按各个线组成新的有序的点云
    // pcl::PointCloud<PointType>::Ptr cloudByScans_ptr;
    if (cloudByScans_ptr->points.size() != 0)
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
        // 如果刚好跨过一条线这个曲率计算是否有问题？ 
        double diffX = cloudByScans_ptr->points[i - 5].x + cloudByScans_ptr->points[i - 4].x + cloudByScans_ptr->points[i - 3].x + cloudByScans_ptr->points[i - 2].x + cloudByScans_ptr->points[i - 1].x - 10 * cloudByScans_ptr->points[i].x + cloudByScans_ptr->points[i + 1].x + cloudByScans_ptr->points[i + 2].x + cloudByScans_ptr->points[i + 3].x + cloudByScans_ptr->points[i + 4].x + cloudByScans_ptr->points[i + 5].x;
        double diffY = cloudByScans_ptr->points[i - 5].y + cloudByScans_ptr->points[i - 4].y + cloudByScans_ptr->points[i - 3].y + cloudByScans_ptr->points[i - 2].y + cloudByScans_ptr->points[i - 1].y - 10 * cloudByScans_ptr->points[i].y + cloudByScans_ptr->points[i + 1].y + cloudByScans_ptr->points[i + 2].y + cloudByScans_ptr->points[i + 3].y + cloudByScans_ptr->points[i + 4].y + cloudByScans_ptr->points[i + 5].y;
        double diffZ = cloudByScans_ptr->points[i - 5].z + cloudByScans_ptr->points[i - 4].z + cloudByScans_ptr->points[i - 3].z + cloudByScans_ptr->points[i - 2].z + cloudByScans_ptr->points[i - 1].z - 10 * cloudByScans_ptr->points[i].z + cloudByScans_ptr->points[i + 1].z + cloudByScans_ptr->points[i + 2].z + cloudByScans_ptr->points[i + 3].z + cloudByScans_ptr->points[i + 4].z + cloudByScans_ptr->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 这个数组很大
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
        if (scanEndInd[i] - scanStartInd[i] < 6)
            continue;

        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatEachScan_ptr(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++) // 分成6部分
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, [this](int i, int j)
                      { return this->cloudCurvature[i] < this->cloudCurvature[j]; }); // 对该部分进行曲率的排序 排好的索引放在cloudSortInd中

            // 选取sharp的点的数量增加
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; // 全部点云中的索引
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                { // 当前索引中的点没有被选中过，并且曲率足够大
                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2; // 标记为２
                        cornerPointsSharp.push_back(cloudByScans_ptr->points[ind]); // 每段最多放２个点进去
                        cornerPointsLessSharp.push_back(cloudByScans_ptr->points[ind]); // 包含了ｃｏｒｎｅｒ的点
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(cloudByScans_ptr->points[ind]); // 每段最多放20个点进去
                    }
                    else
                    {
                        // 超过２０个点之后就不再继续
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l - 1].x;
                        float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l - 1].y;
                        float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) 
                        {
                            break;
                        }
                        // 如果周围的点的曲率大于一定值则不会被标记
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
                    if (smallestPickedNum >= 4) // 选取四个点
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = cloudByScans_ptr->points[ind + l].x - cloudByScans_ptr->points[ind + l - 1].x;
                        float diffY = cloudByScans_ptr->points[ind + l].y - cloudByScans_ptr->points[ind + l - 1].y;
                        float diffZ = cloudByScans_ptr->points[ind + l].z - cloudByScans_ptr->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) 
                        // TODO 为什么这里也是大于0.05？ 选择平面点的时候不应该是太小的标记吗？
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
                if (cloudLabel[k] <= 0) // 所以大部分都是less flat的点 ，其他三种只有20 4 2 这种级别的数量
                {
                    surfPointsLessFlatEachScan_ptr->push_back(cloudByScans_ptr->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;          // 降采样后的点
        downSizeFilter.setInputCloud(surfPointsLessFlatEachScan_ptr); // 每条线中的lessflat的点被降采样，放入临时对象中
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        surfPointsLessFlat += surfPointsLessFlatScanDS; // 降采样后的点放到less flat的点集中
    }
}

void LidarPreProcess::lidar_callback_func(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg_in)
{

#ifdef DEBUG
    cout << "get lidar data!" << endl;
#endif


    cloudOri.clear();
    
    tcal.tic();

    pcl::fromROSMsg(*pointCloudMsg_in, cloudOri); //该函数的第二个形参只能是pointcloud形式
    std::vector<int> indices;

    // cloudPreProcess.clear();
    // pcl::removeNaNFromPointCloud(cloudOri, cloudOri, indices);    // 先去掉NaN的点 is_dense的情况下不会滤去NaN的点...
    // cout << "是否dense: " << cloudOri.is_dense
    //      << endl;
    // dist_filter(cloudOri, cloudPreProcess);
    cout << "原始点云的大小： " << cloudOri.size() << endl;
    removeClosedPointCloud(cloudOri, cloudOri, MINIMUM_RANGE); // 去掉比较近的点  仍然把点存在ori中
    cout << "remove后点云的大小： " << cloudOri.size() << endl;
    cout << tcal.toc() << endl;

    // // 点云扫描分层
    tcal.tic();
    extract_line(cloudOri);
    cout << tcal.toc() << " -扫描分层时间" << endl;

    // 特征点提取
    tcal.tic();
    extract_feature();
    cout << "提取特征的时间： " << tcal.toc() << endl;

    // 发布特征点点云
    publish_point_cloud(pointCloudMsg_in);
}


void LidarPreProcess::publish_point_cloud(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg_in)
{
     sensor_msgs::PointCloud2 laserCloudOutMsg; // 按scan排好的点云
    pcl::toROSMsg(*cloudByScans_ptr, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = pointCloudMsg_in->header.stamp;
    laserCloudOutMsg.header.frame_id = "/rslidar";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg; // sharp的角点
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = pointCloudMsg_in->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/rslidar";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = pointCloudMsg_in->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/rslidar";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = pointCloudMsg_in->header.stamp;
    surfPointsFlat2.header.frame_id = "/rslidar";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = pointCloudMsg_in->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/rslidar";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // // pub each scam
    if (1)
    {
        for (int i = 0; i < lidarConf.get_num_scan(); i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/process/lines/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
        for (int i = 0; i < lidarConf.get_num_scan(); i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(cloudScans[i], scanMsg);
            scanMsg.header.stamp = pointCloudMsg_in->header.stamp;
            scanMsg.header.frame_id = "/rslidar";
            pubEachScan[i].publish(scanMsg);
        }
    }

}
