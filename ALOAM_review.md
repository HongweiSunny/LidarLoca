# ALOAM_review

## lidar_process

### 模块输入：

​	激光雷达点云sensor_msg::PointCloud2

### 模块输出：

​	处理好的有序点云

### 模块流程：

1. **原始点云P0的距离滤波** :  距离过近（0.3m）或者过远（80m）剔除，nan点剔除，注意PCL自带的removeNaNFromCloud()会先判断点云是否是is_dense，若是，会直接跳过后续的处理步骤，可能会删除不干净所有NaN点，所以可以仿造该函数自己写一个处理NaN点的函数；最终得到P1;
2. **点元有序化**：遍历P1的所有点，计算水平角度和仰角，按照仰角得出该点所处的Lidar线束ID,放入一个vector<PointCloud<PointXYZI>>中，该数组的大小和Lidar线束数目相同；该数组标记为P2;
3. **提取特征**：遍历P3中的点，计算每个点在所处的线束上的曲率；对每条线束上的点的曲率进行排序，按曲率从大到小的规则寻找到一定数量（2）的点的索引，做为corner特征，继续往下找找到一定数量（20）的less corner点；反之找到flat点，剩下的点作为less flat点；最终输出四种曲率范围对应的4个特征点云，并发布给Lidar_odom节点用于特征匹配；

### 一些细节：

 * 每条线都按照６部分来提取特征

 * less sharp是包含sharp的点的，sharp点被标记为２，less sharp 被标记为１，

 * 被标记为corner的点，若周围的１０个点的曲率没有超过设定的阈值，要被标记为选中状态

 * ４个flat的点，被标记为－１

 * 所有标记为-1或者０的点，都被加入到less_flat 中

 * 0.1 是sharp和flat的分界点

 * less_flat的点被降采样器降采样

   

### 待改进：





## lidar_odometry

### 模块输入：

​	特征提取模块提取出来的点云；

### 模块输出：

​	里程计信息，激光雷达相对于odom坐标系而言的位姿；

### 模块流程：

主函数中按照顺序，先收取一次msg信息，再进行一次里程计处理，之后也许可以双线程处理，需要加一些线程锁；

```
	while (ros::ok())
    {
        ros::spinOnce(); // 收取一次ros msg
        // lidarOdom_node.spin_once(); // 处理一次数据
        LO.spin_once(); // 处理一次数据
    }
```

进入LO.spin_once 之后，首先判断点云的缓存区是否为空，若为空，则退出；

再检查每个点云缓存区的第一个的秒级的时间是不是相同的，用以判断是否同步，不同步，则直接退出ROS系统；

若前两步都符合要求，则进行非线性优化问题ＮＬＰ的求解；

NLP:

 1. 特征关联方法：

    * 把上一帧less_corner的点云构建成一个kdTree;

    * 找到本次处理的less corner特征点云，遍历每个点；

      * 消除每个点的畸变，具体方法为：假设车辆是匀速运动的，通过上一个点云处理节点中得到的时间信息，变换点云的坐标到该次扫描的最开始处，代码如下：

        ```
        void LidarOdom::TransformToStart(PointType const *const pi, PointType *const po)
        {
            //interpolation ratio
            double s;
            if (DISTORTION)　// 是否处理畸变的宏定义
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
        ```

        

	2. 损失函数计算方法：
 	3. 优化方法
 	4. 
