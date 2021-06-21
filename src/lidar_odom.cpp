#include "lidar_odom.h"

void LidarOdom::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock(); // 防止此时跳出这个回调函数
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void LidarOdom::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void LidarOdom::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void LidarOdom::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void LidarOdom::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{ // 把scan register中的全部点云放进缓冲区中
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

bool LidarOdom::notempty_check()
{
    bool res = !cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() && !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty();

    return res;
}

void LidarOdom::synchronization_check()
{
    // 找到最前面的数据的秒级别的数据 看是否一致 有一个不一致就会中断ROS
    timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec(); // 这些变量后面发布信息时会用到
    timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
    timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
    timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
    timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

    if (timeCornerPointsSharp != timeLaserCloudFullRes || timeCornerPointsLessSharp != timeLaserCloudFullRes || timeSurfPointsFlat != timeLaserCloudFullRes || timeSurfPointsLessFlat != timeLaserCloudFullRes)
    {
        printf("BREAK:   Unsync messeage! ——> In laser_odometry.cpp\n");
        ROS_BREAK();
    }
    // return res;
}

void LidarOdom::copy_feature_point_clout()
{
    std::lock_guard<std::mutex> lockGuard(mBuf);

    cornerPointsSharp->clear();
    pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
    cornerSharpBuf.pop();
    cornerPointsLessSharp->clear();
    pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
    cornerLessSharpBuf.pop();
    surfPointsFlat->clear();
    pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
    surfFlatBuf.pop();
    surfPointsLessFlat->clear();
    pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
    surfLessFlatBuf.pop();
    laserCloudFullRes->clear();
    pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
    fullPointsBuf.pop();
}

// process pointcloud
void LidarOdom::spin_once()
{
    // 1.1 缓冲区非空检测
    if (!notempty_check())
    {
        rate.sleep();
        continue;
    }
    //　1.2 同步检测
    synchronization_check();

    //  2. 内部带安全锁，拷贝msg中的点云
    copy_feature_point_clout();

    //  3. 求解非线性优化问题
    TicToc t_whole;
    t_whole.tic();
    // initializing
    if (!systemInited)
    {
        systemInited = true;
        std::cout << "Initialization finished \n";
    }
    else
    {
        cout << "nlp 优化求解\n";
        calculate_pose_nlp(); //上一次计算出来的结果作为初值
    }
    cout << "总时间： " << t_whole.toc() << " ms " << endl;

    // // 后处理
    after_process();

    // // 发布结果
    pub_result();

} // end of func

// 3.1 NLP 优化函数
void LidarOdom::calculate_pose_nlp()
{
    TicToc t_opt;
    // 最大迭代次数是２次
    for (size_t opti_counter = 0; opti_counter < MAX_ITERATION; ++opti_counter)
    {
        corner_correspondence = 0;
        plane_correspondence = 0;

        //ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 鲁棒核函数
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        // 优化7个参数
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);

        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        TicToc t_data;
        // find correspondence for corner features
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        // 1. sharp点找关联线
        for (int i = 0; i < cornerPointsSharpNum; ++i)
        { // 每个点都找上一帧中相关联的特征点 加入ceres的残差块
            // cout << cornerPointsSharp->points[i].intensity << endl;
            TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); // 找到距离最近的点

            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) // 25米的阈值
            {
                // 最近点索引
                closestPointInd = pointSearchInd[0];

                // 最近点所在的线的ID
                int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                {
                    // if in the same scan line, continue
                    if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID) // 如果点的线号小于等于最近点的线号 继续找
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN)) // 2.5 的常量 // 不能找 太远的点
                        break;

                    double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2) // 在上一帧的特征点中找到的kdtree最近点的附近线束上找到的点满足阈值条件，
                    {
                        // find nearer point
                        // 更新最近点的信息和索引
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if in the same scan line, continue
                    if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break; // 点都是按照线的顺序有序放的

                    double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2)
                    {
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }
            }
            // both closestPointInd and minPointInd2 is valid 找到了kd树中的最近点a和最近点附近的最近点b
            if (minPointInd2 >= 0)
            {
                Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                           cornerPointsSharp->points[i].y,
                                           cornerPointsSharp->points[i].z);
                Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                             laserCloudCornerLast->points[closestPointInd].y,
                                             laserCloudCornerLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                             laserCloudCornerLast->points[minPointInd2].y,
                                             laserCloudCornerLast->points[minPointInd2].z);

                double s;
                if (DISTORTION)
                    s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                else
                    s = 1.0;

                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s); // 仿函数
                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);                                  // 增加残差块
                corner_correspondence++;                                                                                 // corner关联项加1
            }
        }

        // 2. flat点找关联面
        // find correspondence for plane features
        int surfPointsFlatNum = surfPointsFlat->points.size();
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {
            TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // 2.1  search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                    {                                // 因为点是按照线的顺序放的，所以这里表示第二个点只能在同一个线上找到  不会进入lower scan
                        minPointSqDis2 = pointSqDis; // 为什么没有无视这个点
                        minPointInd2 = j;
                    }
                    // if in the higher scan line
                    else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                // 2.2 search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0) // abc三个点就很神奇
                {
                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                               surfPointsFlat->points[i].y,
                                               surfPointsFlat->points[i].z);
                    Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                 laserCloudSurfLast->points[closestPointInd].y,
                                                 laserCloudSurfLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                 laserCloudSurfLast->points[minPointInd2].y,
                                                 laserCloudSurfLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                 laserCloudSurfLast->points[minPointInd3].y,
                                                 laserCloudSurfLast->points[minPointInd3].z);

                    double s;
                    if (DISTORTION)
                        s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    plane_correspondence++;
                }
            }
        }

        printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
        printf("data association time %f ms \n", t_data.toc());

        if ((corner_correspondence + plane_correspondence) < 10)
        {
            printf("less correspondence! *************************************************\n");
        }

        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        printf("solver time %f ms \n", t_solver.toc());
    }
    printf("MAX_ITRATION- %d : optimization twice time %f \n", MAX_ITERATION, t_opt.toc());

    t_w_curr = t_w_curr + q_w_curr * t_last_curr; // 认为现在 旋转上没变化 t_last_curr 是两帧之间的变化，q*t得到的是增量在w坐标系下的投影表示
    q_w_curr = q_w_curr * q_last_curr;            //再去更新旋转

} // end of func

// 更新对象的状态
void LidarOdom::after_process()
{
    // 交换了点，last的变成队列中最前面的  用于更新Kd树
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast; // 都是less的
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

#ifdef DEBUG
    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';
#endif
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*laserCloudSurfLast, *laserCloudSurfLast, indices2);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    // printf("publication time %f ms \n", t_pub.toc());
    // printf("whole laserOdometry time %f ms \n \n", t_whole.toc());

    // 超时警告信息
    // if (t_whole.toc() > 100)
    // {
    //     ROS_WARN("odometry process over 100ms");
    // }
} // end of func

void LidarOdom::pub_result()
{
    // publish odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/odom";
    laserOdometry.child_frame_id = "/rslidar";
    laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat); // 这个时间在初始化的时候是收到的上一个节点的第一帧的结果
    laserOdometry.pose.pose.orientation.x = q_w_curr.x();                     // current 的四元数
    laserOdometry.pose.pose.orientation.y = q_w_curr.y();
    laserOdometry.pose.pose.orientation.z = q_w_curr.z();
    laserOdometry.pose.pose.orientation.w = q_w_curr.w();
    laserOdometry.pose.pose.position.x = t_w_curr.x();
    laserOdometry.pose.pose.position.y = t_w_curr.y();
    laserOdometry.pose.pose.position.z = t_w_curr.z();
    pubLaserOdometry.publish(laserOdometry);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header = laserOdometry.header;
    laserPose.pose = laserOdometry.pose.pose;
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = "/odom";
    pubLaserPath.publish(laserPath);

    // 下面是为建图节点提供输入
    if (frameCount % skipFrameNum == 0) // skip 参数给的1，也就是每次都会发送给第三个节点，如果给的是2的话，后面建图的频率就会是5Hz
    {
        frameCount = 0;
        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2); // 发布此时的上一帧点云 其实就是发布刚刚获取到的队列中的特征
        laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudCornerLast2.header.frame_id = "/rslidar"; // 2代表现在做odom的时候用的less corner特征
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudSurfLast2.header.frame_id = "/rslidar";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2); // 2代表现在做odom的时候用的less surf特征

        sensor_msgs::PointCloud2 laserCloudFullRes3; // 给第三个节点的被scan register处理后的全部点云
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudFullRes3.header.frame_id = "/rslidar";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
    }
    frameCount++;
} // end of func

// undistort lidar point
void LidarOdom::TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD; // intensity中装的是每个点的扫描时间
    else
        s = 1.0;

    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr); // 旋转插值 Quaternion.identity就是指Quaternion(1,0,0,0),
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last; // 从旧系到新系是先平移再旋转  但是这里应该是从新的到旧的，先旋转再平移了
                                                                    //  平移是发生了当前坐标系下
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void LidarOdom::TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}
