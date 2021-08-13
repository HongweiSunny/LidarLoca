# ALOAM_review

[TOC]



## lidar_process

### 模块输入：

​	激光雷达点云sensor_msg::PointCloud2

### 模块输出：

​	处理好的有序点云

### 模块流程：

#### 1. 原始点云P0的距离滤波 :  

​	距离过近（0.3m）或者过远（80m）剔除，nan点剔除，注意PCL自带的removeNaNFromCloud()会先判断点云是否是is_dense，若是，会直接跳过后续的处理步骤，可能会删除不干净所有NaN点，所以可以仿造该函数自己写一个处理NaN点的函数；最终得到P1;

#### 2. 点元有序化：

​	遍历P1的所有点，计算水平角度和仰角，按照仰角得出该点所处的Lidar线束ID,放入一个vector<PointCloud<PointXYZI>>中，该数组的大小和Lidar线束数目相同；该数组标记为P2;

#### 3. 提取特征：

​	遍历P3中的点，计算每个点在所处的线束上的曲率；对每条线束上的点的曲率进行排序，按曲率从大到小的规则寻找到一定数量（2）的点的索引，做为corner特征，继续往下找找到一定数量（20）的less corner点；反之找到flat点，剩下的点作为less flat点；最终输出四种曲率范围对应的4个特征点云，并发布给Lidar_odom节点用于特征匹配；

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

若前两步都符合要求，则进行非线性优化问题NLP的求解，如下；

**NLP:**

#### 1. 特征关联方法：

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

  * 找上一帧中的关联直线
  
    在kdtree中按照距离寻找离当前去畸变变换后得到的点最近的１个点，作为关联直线的第一个点；
  
    在最近点附近的３个线束中找离最近点最近的点，作为关联直线的第二个点；
  
  * 增加NLP问题的点线距离残差块
  
* 找到本次处理的less flat的点云构建一个kdtree

  * 点云去畸变处理，变换插值

  * 寻找关联平面

    在kdtree中按照距离寻找离当前去畸变变换后得到的点最近的１个点，作为关联平面的第一个点；

    在最近点附近的更高编号的３个线束中找离最近点最近的点，作为关联平面的第二个点；

    在最近点附近的更低编号的３个线束中找离最近点最近的点，作为关联平面的第三个点；

   * 增加点面距离残差块

#### 2. 求解与更新成员变量

```
t_w_curr = t_w_curr + q_w_curr * t_last_curr; 
// 认为现在 旋转上没变化 t_last_curr 是两帧之间的变化，q*t得到的是增量在w坐标系下的投影表示
q_w_curr = q_w_curr * q_last_curr;
// 这里的旋转和平移是基于world坐标系表示的
// 更新后的旋转定位信息可用于发布至ros中
```



## lidar_mapping

### 模块输入：

​	激光雷达点云sensor_msg::PointCloud2

​	odom节点的odometry的topic

### 模块输出：

​	一定范围内的局部地图

​	修正后的车辆位姿

### 模块流程：

#### 1. 双线程：

​	双线程，主线程处理收到的topic，放入缓存区；

​	另一个线程负责进行地图匹配；

```
int main(int argc, char** argv)
{
    cout << "Hello Mapping/n";

    ros::init(argc, argv, "lidar_mapping_node");

    LidarMapping lidarMap;

    //    std::thread mapping_process_th(process_map, lidarMap);  
    // process_map(lidarMap);

    std::thread map_th(process_map_func, std::ref(lidarMap));　// 单独弄了个线程
    ros::spin();
    map_th.join();

    return 0;
}
```

#### ２．线程１——处理消息

##### laserOdometry的消息

```
//receive odomtry
void LidarMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    // 线程锁
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
    // wmap_wodom 是建图过程中对wdom做的修正
    // mapping 节点一直维护的是map到odom之间的关系
    // 用４*4的旋转矩阵的方式来理解下面的转换关系
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 
  

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
    
    // =========================================================================================
    //  直接发布是因为建图的频率是比较低的，这样可以使得上一次的建图优化的结果可以按照收到的odometry的频率被利用
}
```

其他三个点云相关的消息都被放如一个消息队列queue中

```
void LidarMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
    mBuf.lock();
    cornerLastBuf.push(laserCloudCornerLast2);
    mBuf.unlock();
}
// std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
```



#### 3. 线程２-mapping

**大循环：**

while (1)

 {

​	while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty())

​    {	//  ------------------ 非空

​		// 取得互斥量

​		mbuf.lock();

 		// 检测时间同步

​	　a. sync_to_corner_detect();

​		b. sync_cloud_time_detect();



​	 	// 取出队列中的点云和doometry放入ｑ_wdom_curr、t_wodom_curr

​		ｃ. take_cloud_from_buf();

​	　 //  这里把cornerBuf清空，前面的surf和fullres都要和它同步，时间慢于corner的都会被drop掉

​            while (!cornerLastBuf.empty())

​            {

​                cornerLastBuf.pop();

​                printf("drop lidar frame in mapping for real time performance \n");

​            }

​       // 释放互斥量

​		mbuf.unlock();

​		ｄ. 设置map坐标系下的初始位姿估计

​		transformAssociateToMap();

​		e. 计算当前雷达处于局部地图的哪个cube

```
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
```

​		f. 拓展局部大地图

​			局部大地图是用一个数组来管理的，数组的大小是固定的，也按照corner和surf分为两个数组

```
	extend_cubes(centerCubeI, centerCubeJ, centerCubeK);
	get_cloud_from_cubes(centerCubeI, centerCubeJ, centerCubeK);
```

​		g. 获取局部小地图

​		把各个轴的正负的两个cube中的点元取出组成一个点云

​		h. 降采样用于匹配的点云

​		I. 关联特征

​			构建surf树和corner的kd树;

​			遍历当前帧点云，变换到ｗmap坐标系下面，寻找树中最近的５个点

​	}

}

**详细：**

a. 时间戳和corner的点云对齐

```
bool LidarMapping::sync_to_corner_detect()
{
    bool flag = true;

    // // odometryBuf只保留一个与cornerLastBuf.front()时间同步的最新消息
    //  非空并且最前面的时间戳小于最新收到的odom的时间戳，就一直弹出，直到弹到空了，或者最前面的时间戳大于等于收到的点云的时间戳了
    while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
        odometryBuf.pop();
    if (odometryBuf.empty()) // 如果是因为弹空了 退出的循环， 说明没找到符合要求的odom的信息，
    {
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
```

b. 检测四个消息的时间戳是否相同

```
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
```

c. 

ｄ.

```
// set initial guess
void LidarMapping::transformAssociateToMap()
{
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}
```

ｇ.

```
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
    // 
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
```



## 评估

```python
evo_traj kitti KITTI_00_ORB.txt KITTI_00_SPTAM.txt --ref=KITTI_00_gt.txt -p --plot_mode=xz
```

## Q

* 为什么ｏｄｏｍ在特征关联的时候不能找同一个扫描的点

## HowTo

* how to 转换四元数到旋转矩阵

  ```
  tf::Quaternion quat;
  tf::quaternionMsgToTF(rtk_msgs->pose.orientation, quat);
  tf::Matrix3x3 T = tf::Matrix3x3(quat);
  ```

## 可调参数

* 是否去畸变
* 最大迭代次数
* 特征点选取的数量

## 小结

* 适配ALOAM算法与pandar128线激光雷达

  * 提取激光雷达的128线束角度的标定结果

  * 通过标定的128个线束的角度分割线束
  * 删除原始点云中距离过远的点（>100m）
  * 原始点云降采样（0.2m采样精度）

* 编写订阅rtk定位结果和aloam定位结果的节点
  * 将结果写入对应的txt文件
  * 使用evo工具评估算法结果

* 结果为什么仍然不是很好？？？

![evo_traj_xyz](/home/nio/pic/evo_traj_xyz.png)

![evo_rpe](/home/nio/pic/evo_rpe.png)

![aloam_ori_210623](/home/nio/pic/aloam_ori_210623.png)

