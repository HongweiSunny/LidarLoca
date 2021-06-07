#include "lidar_mapping.h"

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
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; // 为什么 t_w_curr 和 t_wdom_curr不是在同一个坐标系下面的？？？TODO

    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/wmap"; //  "/camera_init";
    odomAftMapped.child_frame_id = "/wodom"; // "/apt_mapped"
    odomAftMapped.header.stamp = laserOdometry->header.stamp;
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    odomAftMapped.pose.pose.position.x = t_w_curr.x();
    odomAftMapped.pose.pose.position.y = t_w_curr.y();
    odomAftMapped.pose.pose.position.z = t_w_curr.z();
    pubOdomAftMappedHighFrec.publish(odomAftMapped);
}

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
} // 设置初值  还是没弄清楚到底是新系到旧系？？？ 还是反过来？？

void LidarMapping::transformUpdate()
{
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse(); // q_w_curr 计算完成之后，更新当前帧对应的优化后的pose和odometry计算出来的q_wodom_curr之间的增量
    // 给下一帧点云使用
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

// 线程函数入口
void process_map_func(LidarMapping &LM)
{
    LM.process();
}



