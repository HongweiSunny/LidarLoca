// 用于结果评估的一个程序
// 接收

// 高频率的发布
// pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/mapping/aft_mapped_to_init_high_frec", 100);

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <queue>
#include <string>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h" //转换函数头文件

using namespace std;

void aftMappedNav_handler(const nav_msgs::OdometryConstPtr &aftMappedNav_msg);
void odom_handler(const nav_msgs::OdometryConstPtr &odom_msg);
void quternion_2_matrix();
void rtk_handler(const geometry_msgs::PoseStampedConstPtr &rtk_msgs);

// std::queue<Eigen::Matrix4f> Q_rtk;
// std::queue<Eigen::Matrix4f> Q_aloam;

class WriteTumTraj
{

public:
    string fileName;
    ofstream of;
    int count_rtk = 0;
    Eigen::Matrix4f T_rtk_first; // 第一帧的RTK信息

    // 两个标定矩阵
    Eigen::Matrix4f T_inn_pandar;
    Eigen::Matrix4f T_inn_imu;
    Eigen::Matrix4f T_pandar_imu;
    Eigen::Matrix4f T_imu_pandar;

public:
    WriteTumTraj(string fileName_) : fileName(fileName_), of(fileName, ios_base::out)
    {
        cout << "ready to write to:" << fileName << endl;
        T_inn_pandar(0, 0) = -0.00257508870640393;
        T_inn_pandar(0, 1) = -0.9999843178123534;
        T_inn_pandar(0, 2) = 0.004973233226948262;
        T_inn_pandar(0, 3) = -1.941744241937233;

        T_inn_pandar(1, 0) = 0.9999517286601256;
        T_inn_pandar(1, 1) = -0.0025285238002312254;
        T_inn_pandar(1, 2) = 0.00949457305906761;
        T_inn_pandar(1, 3) = 0.028259243456309216;

        T_inn_pandar(2, 0) = -0.009481956095560632;
        T_inn_pandar(2, 1) = 0.004996625299238999;
        T_inn_pandar(2, 2) = 0.9999425614728579;
        T_inn_pandar(2, 3) = 1.9060467166653206;

        T_inn_pandar(3, 0) = 0.;
        T_inn_pandar(3, 1) = 0.;
        T_inn_pandar(3, 2) = 0.;
        T_inn_pandar(3, 3) = 1.;

        T_inn_imu(0, 0) = 1.;
        T_inn_imu(0, 1) = 0.;
        T_inn_imu(0, 2) = 0.;
        T_inn_imu(0, 3) = -3.005;

        T_inn_imu(1, 0) = 0.;
        T_inn_imu(1, 1) = 1.;
        T_inn_imu(1, 2) = 0.;
        T_inn_imu(1, 3) = 0.;

        T_inn_imu(2, 0) = 0.;
        T_inn_imu(2, 1) = 0.;
        T_inn_imu(2, 2) = 1.;
        T_inn_imu(2, 3) = 0.525;

        T_inn_imu(3, 0) = 0;
        T_inn_imu(3, 1) = 0;
        T_inn_imu(3, 2) = 0;
        T_inn_imu(3, 3) = 1;

        // T_inn_imu = Eigen::Matrix4f::Identity();

        T_pandar_imu = T_inn_pandar.inverse() * T_inn_imu;
        T_imu_pandar = T_pandar_imu.inverse();
    }
    WriteTumTraj() = default;

    void write_pose(const geometry_msgs::PoseStampedConstPtr &poseMsg)
    {
        of << poseMsg->header.stamp << " "
           << poseMsg->pose.position.x << " "
           << poseMsg->pose.position.y << " "
           << poseMsg->pose.position.z << " "
           << poseMsg->pose.orientation.x << " "
           << poseMsg->pose.orientation.y << " "
           << poseMsg->pose.orientation.z << " "
           << poseMsg->pose.orientation.w
           << endl;
    }
    void write_pose(const nav_msgs::OdometryConstPtr &poseMsg)
    {
        of << poseMsg->header.stamp << " "
           << poseMsg->pose.pose.position.x << " "
           << poseMsg->pose.pose.position.y << " "
           << poseMsg->pose.pose.position.z << " "
           << poseMsg->pose.pose.orientation.x << " "
           << poseMsg->pose.pose.orientation.y << " "
           << poseMsg->pose.pose.orientation.z << " "
           << poseMsg->pose.pose.orientation.w
           << endl;
    }
    void write_pose(const geometry_msgs::PoseStampedConstPtr &poseMsg, const int &a) // 形参为转换好的旋转矩阵和平移矩阵
    {

        count_rtk++;
        // of<<count_rtk<<endl;
        Eigen::Matrix4f T_rtk_now = pack_T(poseMsg);
        if (count_rtk == 1)
        {
            // 初始化第一帧rtk坐标
            T_rtk_first = T_rtk_now;
        }
        // if(count_rtk == 1)
        // {
        //     return;
        // }

        Eigen::Matrix4f T_pandar_real = T_pandar_imu * (T_rtk_first.inverse() * T_rtk_now) * T_imu_pandar;


        // T_pandar_real = homo_T.inverse() * T_pandar_real * homo_T;

        unpack_T(T_pandar_real, poseMsg);
    }

    Eigen::Matrix4f pack_T(const geometry_msgs::PoseStampedConstPtr &poseMsg)
    {

        float x = poseMsg->pose.orientation.x;
        float y = poseMsg->pose.orientation.y;
        float z = poseMsg->pose.orientation.z;
        float w = poseMsg->pose.orientation.w;
        Eigen::Quaterniond q(w, x, y, z);
        Eigen::Matrix3d R = q.toRotationMatrix();
        float a = poseMsg->pose.position.x;
        float b = poseMsg->pose.position.y;
        float c = poseMsg->pose.position.z;
        Eigen::Vector3f t(a, b, c);

        Eigen::Matrix4f T;
        T(0, 0) = R(0, 0);
        T(0, 1) = R(0, 1);
        T(0, 2) = R(0, 2);

        T(1, 0) = R(1, 0);
        T(1, 1) = R(1, 1);
        T(1, 2) = R(1, 2);

        T(2, 0) = R(2, 0);
        T(2, 1) = R(2, 1);
        T(2, 2) = R(2, 2);

        T(0, 3) = t(0);
        T(1, 3) = t(1);
        T(2, 3) = t(2);

        T(3, 0) = 0.;
        T(3, 1) = 0.;
        T(3, 2) = 0.;
        T(3, 3) = 1.;

        return T;
    }

    void unpack_T(Eigen::Matrix4f &T, const geometry_msgs::PoseStampedConstPtr &poseMsg)
    {
        static tf::TransformBroadcaster br;
        // 从T中计算出四元数和评议向量
        float qw = sqrt(1 + T(0, 0) + T(1, 1) + T(2, 2)) / 2.;
        float qx = (T(2, 1) - T(1, 2)) / 4. / qw;
        float qy = (T(0, 2) - T(2, 0)) / 4. / qw;
        float qz = (T(1, 0) - T(0, 1)) / 4. / qw;
        of << poseMsg->header.stamp << " "
           << T(0, 3) << " "
           << T(1, 3) << " "
           << T(2, 3) << " "
           << qx << " "
           << qy << " "
           << qz << " "
           << qw << endl;

        // 发布tf
        float a = T(0, 3);
        float b = T(1, 3);
        float c = T(2, 3);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(a, b, c));
        tf::Quaternion qq(qx, qy, qz, qw);
        transform.setRotation(qq);

        br.sendTransform(tf::StampedTransform(transform, poseMsg->header.stamp, "/map", "/rtkPandar"));
    }

    ~WriteTumTraj()
    {
        of.close();
    }
};

WriteTumTraj rtkWriter("/home/nio/catkin_ws_my/src/LidarLoca/rtk.txt");
WriteTumTraj rtkPandarWriter("/home/nio/catkin_ws_my/src/LidarLoca/rtkPandar.txt");

WriteTumTraj aloamWriter("/home/nio/catkin_ws_my/src/LidarLoca/aftmap.txt");
WriteTumTraj odomWriter("/home/nio/catkin_ws_my/src/LidarLoca/odom.txt");

// /odom/laser_odom_path

int main(int argc, char **argv)
{
    cout << "Hello world" << endl;

    ros::init(argc, argv, "result_evaluation");

    ros::NodeHandle nh;

    ros::Subscriber subOdomPath = nh.subscribe<nav_msgs::Odometry>("/odom/laser_odom_to_init", 1000, odom_handler);
    ros::Subscriber subAftMappedNav = nh.subscribe<nav_msgs::Odometry>("/mapping/aft_mapped_to_init_high_frec", 1000, aftMappedNav_handler);
    ros::Subscriber subRTK = nh.subscribe<geometry_msgs::PoseStamped>("rtk_ins", 1000, rtk_handler); //<geometry_msgs/PoseStamped>

    ros::spin();

    //rtk_out_file.close();

    return 0;
}

void aftMappedNav_handler(const nav_msgs::OdometryConstPtr &aftMappedNav_msg)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.stamp = aftMappedNav_msg->header.stamp;
    // aloamWriter.write_pose(aftMappedNav_msg);
    aloamWriter.write_pose(aftMappedNav_msg);
}

void rtk_handler(const geometry_msgs::PoseStampedConstPtr &rtk_msgs)
{
    // writing s
    // tum 的格式: time x y z qx qy qz qw

    // 转换一下rtk的坐标 再写入
    // 先从

    // rtkWriter.write_pose(rtk_msgs, 1);
    // rtkPandarWriter.write_pose(rtk_msgs, 1);
    rtkWriter.write_pose(rtk_msgs);
    rtkPandarWriter.write_pose(rtk_msgs, 1);
}

void odom_handler(const nav_msgs::OdometryConstPtr &odom_msg)
{
    nav_msgs::Odometry odomPath;
    odomPath.header.stamp = odom_msg->header.stamp;
    // aloamWriter.write_pose(aftMappedNav_msg);
    odomWriter.write_pose(odom_msg);
}
