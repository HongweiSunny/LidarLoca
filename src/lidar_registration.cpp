#include <iostream>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "lidar_process.h"

using namespace std;

int main(int argc, char** argv)
{
    #ifdef DEBUG
        cout << "Hello world"<<endl;
    #endif
    ros::init(argc,  argv, "lidarProcess");
    
    // LidarQujibian lidarQu;

    LidarPreProcess lidarPro;   // 预处理的类 包括了：距离预处理   去畸变预处理



    ros::spin();

    return 0;
}

