#include "lidar_mapping.h"

// -------------- 四个消息处理函数
void LidarMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
    mBuf.lock();
    cornerLastBuf.push(laserCloudCornerLast2);
    mBuf.unlock();
}

void LidarMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
    mBuf.lock();
    surfLastBuf.push(laserCloudSurfLast2);
    mBuf.unlock();
}

void LidarMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullResBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

//receive odomtry
void LidarMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    mBuf.lock();
    odometryBuf.push(laserOdometry);
    mBuf.unlock();

    // high frequence publish
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x; // x() 返回的是一个引用，所以可以这样赋值
    q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
    q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
    q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
    t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
    t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
    t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

    // wodom_curr 是前端发过来的
    // wmap 是建图过程中对wdom做的修正
    // mapping 节点一直维护的是map到odom之间的关系
    // 用４*4的旋转矩阵的方式来理解下面的转换关系
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/map";    //  "/camera_init";
    odomAftMapped.child_frame_id = "/rslidar"; // "/apt_mapped"
    odomAftMapped.header.stamp = laserOdometry->header.stamp;
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    odomAftMapped.pose.pose.position.x = t_w_curr.x();
    odomAftMapped.pose.pose.position.y = t_w_curr.y();
    odomAftMapped.pose.pose.position.z = t_w_curr.z();
    pubOdomAftMappedHighFrec.publish(odomAftMapped);
    //  直接发布是因为建图的频率是比较低的，这样可以使得上一次的建图优化的结果可以按照收到的odometry的频率被利用
}

// 两个同步检测函数
bool LidarMapping::sync_to_corner_detect()
{
    bool flag = true;

    // // odometryBuf只保留一个与cornerLastBuf.front()时间同步的最新消息
    //  非空并且最前面的时间戳小于最新收到的odom的时间戳，就一直弹出，直到弹到空了，或者最前面的时间戳大于等于收到的点云的时间戳了
    while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
        odometryBuf.pop();
    if (odometryBuf.empty()) // 如果是因为弹空了 退出的循环， 说明没找到符合要求的odom的信息，则解锁，重新进入大循环
    {
        //mBuf.unlock();
        flag = false;
        return flag;
    }

    while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
        surfLastBuf.pop();
    if (surfLastBuf.empty())
    {
        flag = false;
        return flag;
    }

    while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
        fullResBuf.pop();
    if (fullResBuf.empty())
    {
        flag = false;
        return flag;
    }

    return flag;
}

bool LidarMapping::sync_cloud_time_detect()
{
    bool flag = true;
    timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
    timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
    timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
    timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

    if (timeLaserCloudCornerLast != timeLaserOdometry ||
        timeLaserCloudSurfLast != timeLaserOdometry ||
        timeLaserCloudFullRes != timeLaserOdometry)
    {
        flag = false;
        return flag;
    }

    return flag;
}

// 测试线程函数
void process_map_test(LidarMapping &LM)
{
    int a = 0;
    while (a < 10)
    {
        cout << "pp " << LM.laserCloudNum << endl; //
        LM.laserCloudNum++;
        cout << "pp" << endl;
        ++a;
    }
    return;
}

// set initial guess
void LidarMapping::transformAssociateToMap()
{
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void LidarMapping::transformUpdate()
{
    // q_w_curr 计算完成之后，更新当前帧对应的优化后的pose和odometry计算出来的q_wodom_curr之间的增量
    // 给下一帧点云使用
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

// 用Mapping的位姿w_curr，将Lidar坐标系下的点变换到world坐标系下
void LidarMapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void LidarMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    po->intensity = pi->intensity;
}

void LidarMapping::take_cloud_from_buf()
{
    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
    cornerLastBuf.pop();

    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
    surfLastBuf.pop();

    laserCloudFullRes->clear();
    pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
    fullResBuf.pop();
}

void LidarMapping::extend_cubes(int &centerCubeI, int &centerCubeJ, int &centerCubeK)
{                           // 以下注释部分参照LOAM_NOTED，结合我画的submap的示意图说明下面的6个while loop的作用：要
                            // 注意世界坐标系下的点云地图是固定的，但是IJK坐标系我们是可以移动的，所以这6个while loop
                            // 的作用就是调整IJK坐标系（也就是调整所有cube位置），使得五角星在IJK坐标系的坐标范围处于
                            // 3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向
                            // 四周拓展cube（图中的黄色cube就是拓展的cube）时，index（即IJK坐标）成为负数。
    while (centerCubeI < 3) // 靠近边缘
    {
        for (int j = 0; j < laserCloudHeight; j++) // 另外两个维度
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i >= 1; i--) // 在I方向cube[I]=cube[I-1],清空最后一个空出来的cube，实现IJK坐标系向I轴负方向移动一个cube的
                                    // 效果，从相对运动的角度看是图中的五角星在IJK坐标系下向I轴正方向移动了一个cube，如下面动图所示，所
                                    // 以centerCubeI最后++，laserCloudCenWidth也++，为下一帧Mapping时计算五角星的IJK坐标做准备
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                // 最后的i=0 清空
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear(); 
                laserCloudCubeSurfPointer->clear();
            }
        }
        // lidar在Cubes中的坐标加1；
        centerCubeI++;
        // map坐标系在cube中的坐标加1；
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i < laserCloudWidth - 1; i++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = laserCloudHeight - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j >= 1; j--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j < laserCloudHeight - 1; j++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = laserCloudDepth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k >= 1; k--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k < laserCloudDepth - 1; k++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK--;
        laserCloudCenDepth--;
    }
}

void LidarMapping::get_cloud_from_cubes(int &centerCubeI, int &centerCubeJ, int &centerCubeK)
{
    laserCloudValidNum = 0;
    laserCloudSurroundNum = 0;
    // 向IJ坐标轴的正负方向各拓展2个cube，K坐标轴的正负方向各拓展1个cube，上图中五角星所在的蓝色cube就是当前位置
    // 所处的cube，拓展的cube就是黄色的cube，这些cube就是submap的范围
    // 找出可用的周围的cube
    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
            for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
            {
                if (i >= 0 && i < laserCloudWidth &&
                    j >= 0 && j < laserCloudHeight &&
                    k >= 0 && k < laserCloudDepth) // 如果坐标合法
                {
                    // 记录submap中的所有cube的index，记为有效index
                    laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    laserCloudValidNum++;
                    laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    laserCloudSurroundNum++;
                }
            }
        }
    }

    // 清空地图取出来的点云
    // 将有效index的cube中的点云叠加到一起组成submap的特征点云
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
        *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
    }

    laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
    laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
}

void LidarMapping::add_cloud_to_map()
{
    // TicToc t_add;
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
        pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
        if (pointSel.x + 25.0 < 0)
            cubeI--;
        if (pointSel.y + 25.0 < 0)
            cubeJ--;
        if (pointSel.z + 25.0 < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
            cubeJ >= 0 && cubeJ < laserCloudHeight &&
            cubeK >= 0 && cubeK < laserCloudDepth)
        {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
        }
    }

    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
        pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0)
            cubeI--;
        if (pointSel.y + 25.0 < 0)
            cubeJ--;
        if (pointSel.z + 25.0 < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
            cubeJ >= 0 && cubeJ < laserCloudHeight &&
            cubeK >= 0 && cubeK < laserCloudDepth)
        {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
        }
    }
    // printf("add points time %f ms\n", t_add.toc());
}

void LidarMapping::downsample_cubes()
{
    // 因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样
    // 这个地方可以简单优化一下：如果之前的cube没有新添加点就不需要再降采样
    // TicToc t_filter;
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        int ind = laserCloudValidInd[i]; // get_cloud_from_map中被改变 存放了被取了点云的cube的坐标

        pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
        downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
        downSizeFilterCorner.filter(*tmpCorner);
        laserCloudCornerArray[ind] = tmpCorner; // 在堆上开辟的指针 不会被程序自己释放，智能指针拷贝计数加1

        pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
        downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
        downSizeFilterSurf.filter(*tmpSurf);
        laserCloudSurfArray[ind] = tmpSurf;
    }
    // printf("filter time %f ms \n", t_filter.toc());
}

void LidarMapping::publish_result()
{
    // TicToc t_pub;
    //publish surround map for every 5 frame
    if (frameCount % 5 == 0)
    {
        // 所有可见的地图 50m * 21 = 1050m的范围内
        // laserCloudSurround 是雷达周围几个cube内的点云
        laserCloudSurround->clear();
        for (int i = 0; i < laserCloudSurroundNum; i++) // laserCloudSurroundNum 在get_map中被改变
        {
            int ind = laserCloudSurroundInd[i];
            *laserCloudSurround += *laserCloudCornerArray[ind];
            *laserCloudSurround += *laserCloudSurfArray[ind];
        }
        if (pubLaserCloudMap.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
            // sync_cloud_time_detect中取出了时间戳
            laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudSurround3.header.frame_id = "/map"; //"/camera_init";
            pubLaserCloudSurround.publish(laserCloudSurround3);
        }
    }

    // 20帧发布一次全部的大地图
    // 20 * 100ms = 2s
    if (frameCount % 20 == 0)
    {
        frameCount = 0; // 清0
        pcl::PointCloud<PointType> laserCloudMap;
        for (int i = 0; i < laserCloudNum; i++)
        {
            laserCloudMap += *laserCloudCornerArray[i];
            laserCloudMap += *laserCloudSurfArray[i];
        }
        if (pubLaserCloudMap.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudMsg.header.frame_id = "/map";
            pubLaserCloudMap.publish(laserCloudMsg);
        }
    }

    // 把当前收到的点云转移到map坐标系下
    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
        pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
    }

    if (pubLaserCloudFullRes.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/map";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
    }

    // printf("mapping pub time %f ms \n", t_pub.toc());

    // 发布导航信息
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/map";
        odomAftMapped.child_frame_id = "/rslidar";
        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();
        pubOdomAftMapped.publish(odomAftMapped); // 低频的发布

    if (pubLaserAfterMappedPath.getNumSubscribers() != 0)
    {
        geometry_msgs::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = "/map";
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath.publish(laserAfterMappedPath);
    }

    // tf
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_w_curr(0),
                                    t_w_curr(1),
                                    t_w_curr(2)));
    q.setW(q_w_curr.w());
    q.setX(q_w_curr.x());
    q.setY(q_w_curr.y());
    q.setZ(q_w_curr.z());
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/map", "/rslidar"));

    frameCount++;
#ifdef DEBUG
    std::cout << "frameCount: " << frameCount << endl;
#endif
}

void LidarMapping::downsample_cloud_in()
{
    laserCloudCornerStack->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerStack);
    laserCloudCornerStackNum = laserCloudCornerStack->points.size(); // 后面会用到

    laserCloudSurfStack->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfStack);
    laserCloudSurfStackNum = laserCloudSurfStack->points.size(); // 后面会用到
}

// 线程函数入口 不属于类
void process_map_func(LidarMapping &LM)
{
    LM.process();
}
