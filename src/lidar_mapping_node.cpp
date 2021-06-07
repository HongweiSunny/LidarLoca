#include "lidar_mapping.h"

using namespace std;




int main(int argc, char** argv)
{
    cout << "Hello Mapping/n";

    ros::init(argc, argv, "lidar_mapping_node");

    LidarMapping lidarMap;

    //    std::thread mapping_process_th(process_map, lidarMap);  // 单独弄了个线程
    // process_map(lidarMap);

    std::thread map_th(process_map_func, std::ref(lidarMap));
    ros::spin();
    map_th.join();

    return 0;
}