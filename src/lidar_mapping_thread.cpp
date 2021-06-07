#include "lidar_mapping.h"

void LidarMapping::process()
{
    cout << "map_func :\n";
    while (1)
    {
        
        while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty())
        {   // 非空
            // 1. odom时间同步
            bool flag = true;
            mBuf.lock();

            flag = sync_to_corner_detect();
            if (flag == false)
            {
                mBuf.unlock(); // break 之前要解锁
                break;
            }

            // 2. 点云时间同步
            flag = sync_cloud_time_detect();
            if (flag == false)
            {
                printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
                printf("unsync messeage! In lidar_mapping.cpp");
                mBuf.unlock(); // break 之前要解锁
                break;
            }

            // 3. 取出三帧点云
            take_cloud_from_buf();

            // 得到前端发来的wodom_cur
            q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
            q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
            q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
            q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
            t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
            t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
            t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
            odometryBuf.pop();

            //  这里把cornerBuf清空，前面的surf和fullres都要和它同步，时间慢于corner的都会被drop掉
            while (!cornerLastBuf.empty()) 
            {
                cornerLastBuf.pop();
                printf("drop lidar frame in mapping for real time performance \n");
            }

            //  
            mBuf.unlock();

            TicToc t_whole;

            transformAssociateToMap();

            TicToc t_shift;
            // 下面这是计算当前帧位置t_w_curr（在上图中用红色五角星表示的位置）IJK坐标（见上图中的坐标轴），
            // 参照LOAM_NOTED的注释，下面有关25呀，50啥的运算，等效于以50m为单位进行缩放，因为LOAM用1维数组
            // 进行cube的管理，而数组的index只用是正数，所以要保证IJK坐标都是正数，所以加了laserCloudCenWidth/Heigh/Depth
            // 的偏移，来使得当前位置尽量位于submap的中心处，也就是使得上图中的五角星位置尽量处于所有格子的中心附近，
            // 偏移laserCloudCenWidth/Heigh/Depth会动态调整，来保证当前位置尽量位于submap的中心处。
            int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
            int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
            int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

            // 由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
            if (t_w_curr.x() + 25.0 < 0)
                centerCubeI--;
            if (t_w_curr.y() + 25.0 < 0)
                centerCubeJ--;
            if (t_w_curr.z() + 25.0 < 0)
                centerCubeK--;

            // 以下注释部分参照LOAM_NOTED，结合我画的submap的示意图说明下面的6个while loop的作用：要
            // 注意世界坐标系下的点云地图是固定的，但是IJK坐标系我们是可以移动的，所以这6个while loop
            // 的作用就是调整IJK坐标系（也就是调整所有cube位置），使得五角星在IJK坐标系的坐标范围处于
            // 3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向
            // 四周拓展cube（图中的黄色cube就是拓展的cube）时，index（即IJK坐标）成为负数。
            while (centerCubeI < 3)
            {
                for (int j = 0; j < laserCloudHeight; j++)
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
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeI++;
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

            // 向IJ坐标轴的正负方向各拓展2个cube，K坐标轴的正负方向各拓展1个cube，上图中五角星所在的蓝色cube就是当前位置
            // 所处的cube，拓展的cube就是黄色的cube，这些cube就是submap的范围
            int laserCloudValidNum = 0;
            int laserCloudSurroundNum = 0;

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

            laserCloudCornerFromMap->clear();
            laserCloudSurfFromMap->clear();
            for (int i = 0; i < laserCloudValidNum; i++)
            {
                // 将有效index的cube中的点云叠加到一起组成submap的特征点云
                *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
            }
            int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
            int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

            pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
            downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
            downSizeFilterCorner.filter(*laserCloudCornerStack);
            int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

            pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
            downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
            downSizeFilterSurf.filter(*laserCloudSurfStack);
            int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

            printf("map prepare time %f ms\n", t_shift.toc());
            printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
            if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
            {
                TicToc t_opt;
                TicToc t_tree;
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
                printf("build tree time %f ms \n", t_tree.toc());

                for (int iterCount = 0; iterCount < 2; iterCount++)
                {
                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(parameters, 4, q_parameterization);
                    problem.AddParameterBlock(parameters + 4, 3);

                    TicToc t_data;
                    int corner_num = 0;

                    for (int i = 0; i < laserCloudCornerStackNum; i++)
                    {
                        pointOri = laserCloudCornerStack->points[i];
                        // 需要注意的是submap中的点云都是world坐标系，而当前帧的点云都是Lidar坐标系，所以
                        // 在搜寻最近邻点时，先用预测的Mapping位姿w_curr，将Lidar坐标系下的特征点变换到world坐标系下
                        //double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
                        pointAssociateToMap(&pointOri, &pointSel);
                        // 在submap的corner特征点（target）中，寻找距离当前帧corner特征点（source）最近的5个点
                        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                        if (pointSearchSqDis[4] < 1.0)
                        {
                            std::vector<Eigen::Vector3d> nearCorners;
                            Eigen::Vector3d center(0, 0, 0);
                            for (int j = 0; j < 5; j++)
                            {
                                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                    laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                    laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                                center = center + tmp;
                                nearCorners.push_back(tmp);
                            }
                            center = center / 5.0;

                            // 协方差矩阵
                            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                            for (int j = 0; j < 5; j++)
                            {
                                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                            }

                            // 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                            // if is indeed line feature
                            // note Eigen library sort eigenvalues in increasing order
                            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量
                            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) // 如果最大的特征值 >> 其他特征值，则5个点确实呈线状分布，否则认为直线“不够直”
                            {
                                Eigen::Vector3d point_on_line = center;
                                Eigen::Vector3d point_a, point_b;
                                // 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
                                point_a = 0.1 * unit_direction + point_on_line;
                                point_b = -0.1 * unit_direction + point_on_line;

                                // 然后残差函数的形式就跟Odometry一样了，残差距离即点到线的距离，到介绍lidarFactor.cpp时再说明具体计算方法
                                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                                corner_num++;
                            }
                        }
                        /*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
                    }

                    int surf_num = 0;
                    for (int i = 0; i < laserCloudSurfStackNum; i++)
                    {
                        pointOri = laserCloudSurfStack->points[i];
                        //double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
                        pointAssociateToMap(&pointOri, &pointSel);
                        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                        Eigen::Matrix<double, 5, 3> matA0;
                        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                        if (pointSearchSqDis[4] < 1.0)
                        {

                            for (int j = 0; j < 5; j++)
                            {
                                matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                                matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                                matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                                //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                            }
                            // find the norm of plane
                            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                            double negative_OA_dot_norm = 1 / norm.norm();
                            norm.normalize();

                            // Here n(pa, pb, pc) is unit norm of plane
                            bool planeValid = true;
                            for (int j = 0; j < 5; j++)
                            {
                                // if OX * n > 0.2, then plane is not fit well
                                if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                         norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                         norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                                {
                                    planeValid = false;
                                    break;
                                }
                            }
                            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                            if (planeValid)
                            {
                                ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                                surf_num++;
                            }
                        }
                        /*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
                    }

                    //printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
                    //printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

                    printf("mapping data assosiation time %f ms \n", t_data.toc());

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    options.check_gradients = false;
                    options.gradient_check_relative_precision = 1e-4;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    printf("mapping solver time %f ms \n", t_solver.toc());

                    //printf("time %f \n", timeLaserOdometry);
                    //printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
                    //printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
                    //	   parameters[4], parameters[5], parameters[6]);
                }
                printf("mapping optimization time %f \n", t_opt.toc());
            }
            else
            {
                ROS_WARN("time Map corner and surf num are not enough");
            }

            // 完成ICP（迭代2次）的特征匹配后，用最后匹配计算出的优化变量w_curr，更新增量wmap_wodom，为下一次Mapping做准备
            transformUpdate();

            TicToc t_add;
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
            printf("add points time %f ms\n", t_add.toc());

            // 因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样
            // 这个地方可以简单优化一下：如果之前的cube没有新添加点就不需要再降采样
            TicToc t_filter;
            for (int i = 0; i < laserCloudValidNum; i++)
            {
                int ind = laserCloudValidInd[i];

                pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
                downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                downSizeFilterCorner.filter(*tmpCorner);
                laserCloudCornerArray[ind] = tmpCorner;

                pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
                downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                downSizeFilterSurf.filter(*tmpSurf);
                laserCloudSurfArray[ind] = tmpSurf;
            }
            printf("filter time %f ms \n", t_filter.toc());

            TicToc t_pub;
            //publish surround map for every 5 frame
            if (frameCount % 5 == 0)
            {
                laserCloudSurround->clear();
                for (int i = 0; i < laserCloudSurroundNum; i++)
                {
                    int ind = laserCloudSurroundInd[i];
                    *laserCloudSurround += *laserCloudCornerArray[ind];
                    *laserCloudSurround += *laserCloudSurfArray[ind];
                }

                sensor_msgs::PointCloud2 laserCloudSurround3;
                pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
                laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                laserCloudSurround3.header.frame_id = "/camera_init";
                pubLaserCloudSurround.publish(laserCloudSurround3);
            }

            if (frameCount % 20 == 0)
            {
                pcl::PointCloud<PointType> laserCloudMap;
                for (int i = 0; i < 4851; i++)
                {
                    laserCloudMap += *laserCloudCornerArray[i];
                    laserCloudMap += *laserCloudSurfArray[i];
                }
                sensor_msgs::PointCloud2 laserCloudMsg;
                pcl::toROSMsg(laserCloudMap, laserCloudMsg);
                laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                laserCloudMsg.header.frame_id = "/camera_init";
                pubLaserCloudMap.publish(laserCloudMsg);
            }

            int laserCloudFullResNum = laserCloudFullRes->points.size();
            for (int i = 0; i < laserCloudFullResNum; i++)
            {
                pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
            }

            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudFullRes3.header.frame_id = "/camera_init";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);

            printf("mapping pub time %f ms \n", t_pub.toc());

            printf("whole mapping time %f ms +++++\n", t_whole.toc());

            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "/camera_init";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
            odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
            odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
            odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
            odomAftMapped.pose.pose.position.x = t_w_curr.x();
            odomAftMapped.pose.pose.position.y = t_w_curr.y();
            odomAftMapped.pose.pose.position.z = t_w_curr.z();
            pubOdomAftMapped.publish(odomAftMapped);

            geometry_msgs::PoseStamped laserAfterMappedPose;
            laserAfterMappedPose.header = odomAftMapped.header;
            laserAfterMappedPose.pose = odomAftMapped.pose.pose;
            laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
            laserAfterMappedPath.header.frame_id = "/camera_init";
            laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
            pubLaserAfterMappedPath.publish(laserAfterMappedPath);

            static tf::TransformBroadcaster br;
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
            br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

            frameCount++;
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura); // 睡眠两毫秒
    }
}
