#include "lidar_mapping.h"

void LidarMapping::process()
{
    cout << "map_func :\n";
    while (1)
    {

        while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty())
        { // 非空
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

            // 用维护的map_odom * odom_curr　得到map_curr
            transformAssociateToMap(); 

            TicToc t_shift;
            // 下面这是计算当前帧位置t_w_curr（在上图中用红色五角星表示的位置）IJK坐标（见上图中的坐标轴），
            // 参照LOAM_NOTED的注释，下面有关25呀，50啥的运算，等效于以50m为单位进行缩放，因为LOAM用1维数组
            // 进行cube的管理，而数组的index只用是正数，所以要保证IJK坐标都是正数，所以加了laserCloudCenWidth/Heigh/Depth
            // 的偏移，来使得当前位置尽量位于submap的中心处，也就是使得上图中的五角星位置尽量处于所有格子的中心附近，
            // 偏移laserCloudCenWidth/Heigh/Depth会动态调整，来保证当前位置尽量位于submap的中心处。

            // void calculate_IJK();
            // 50是一个cube的size
            // Map原点前后[-25,25]是一个cube
            //  laserCloudCenWidth 是map系处于大地图中的cube坐标，不是常量
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

            extend_cubes(centerCubeI, centerCubeJ, centerCubeK);

            get_cloud_from_cubes(centerCubeI, centerCubeJ, centerCubeK);

            #ifdef DEBUG
                printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
                printf("map prepare time %f ms\n", t_shift.toc());
            #endif

            // 输入点云降采样
            downsample_cloud_in();

            // 优化
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
                        pointOri = laserCloudCornerStack->points[i]; // stack 是被降采样后的点云
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

            // 新的点云加入地图
            add_cloud_to_map();

            // 降采样
            downsample_cubes();

            publish_result();
            
            printf("whole mapping time %f ms +++++\n", t_whole.toc());
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura); // 睡眠两毫秒
    }
}
