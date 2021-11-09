#include "mbot_linux_serial.h"
#include <opencv2/core/core.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h> 
#include <inttypes.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
 
 using namespace std;
struct Imu_Data imu_data;

int main(int argc,char** argv)
{
    std::ofstream f;
    string strSettingPath = string(argv[1]);
    cv::FileStorage fs(strSettingPath, cv::FileStorage::READ);
    float Longitude = fs["imu.Longitude"];
    float Latitude = fs["imu.Latitude"]; 
    string savePath = fs["imu.savepath"];
    bool imu_start_flag;
    imu_start_flag= imu_Init(Longitude,Latitude);

    ros::init(argc,argv,"GD_IMU");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("GD_IMU/imu",200);
    ros::Publisher gt_pub = n.advertise<sensor_msgs::NavSatFix>("GD/NavSatFix",200);
    ros::Rate loop_rate(50);
    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix NavSatFix;
    while (ros::ok()&&imu_start_flag)
    {
        imu_receive(&imu_data);
        imu.header.frame_id = "GD_link";
        imu.header.stamp = ros::Time::now();
        imu.angular_velocity.x = (imu_data.gyro_x);
        imu.angular_velocity.y = (imu_data.gyro_y);
        imu.angular_velocity.z = (imu_data.gyro_z);
        imu.linear_acceleration.y = (imu_data.acc_y);
        imu.linear_acceleration.z = (imu_data.acc_z);
        
         NavSatFix.header.frame_id  = "GD_IMU_link";
         NavSatFix.header.stamp = imu.header.stamp;
        NavSatFix.latitude = imu_data.lt;
        NavSatFix.longitude = imu_data.lg;

        imu_pub.publish(imu);
        gt_pub.publish(NavSatFix);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



void save_imu_data()
{
    // f.open(savePath.c_str());
    // f<<fixed;
    // f<<"acc_x acc_y acc_z gyro_x gyro_y gyro_z lg lt stamp_time"<<endl;
    // cout<<"acc_x = "<<imu_data.acc_x<<endl;
    // cout<<"acc_y = "<<imu_data.acc_y<<endl;
    // cout<<"acc_z = "<<imu_data.acc_z<<endl;
    // f<<imu_data.acc_x<<" "<<imu_data.acc_y<<" "<<imu_data.acc_z<<
    // " "<<imu_data.gyro_x<<" "<<imu_data.gyro_y<<" "<<imu_data.gyro_z<<" "<<imu_data.lg<<" "<<imu_data.lt<<endl;
    // f.close();

}