
#include "lidar_odom.h"

// 线程函数入口 不属于类 调用了类中的方法
void process_odom_func(LidarOdom &LO)
{
    LO.process();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_odometry_node");
    // LidarOdom lidarOdom_node;
    LidarOdom LO;
    cout << "############ start LidarOdom_node!\n";
    // while (ros::ok())
    // {
    //     ros::spinOnce(); // 收取一次ros msg
    //     // lidarOdom_node.spin_once(); // 处理一次数据
    //     LO.spin_once(); // 处理一次数据
    // }

    // 用多线程的方式进行odom的处理
    std::thread odom_th(process_odom_func, std::ref(LO));
    ros::spin();
    odom_th.join();

    return 0;
}
