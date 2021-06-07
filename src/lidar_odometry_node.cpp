
#include "lidar_odom.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_odometry_node");
    // LidarOdom lidarOdom_node;
    LidarOdom LL;
    cout << "start LidarOdom_node!\n";
    while (ros::ok())
    {
        ros::spinOnce(); // 收取一次ros msg
        // lidarOdom_node.spin_once(); // 处理一次数据
        LL.spin_once(); // 处理一次数据
    }
    return 0;
}
