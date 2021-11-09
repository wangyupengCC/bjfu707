#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
using namespace std;
ofstream rgbf;
ofstream depthf;
ofstream leftf;
ofstream rightf;
ofstream imuf;
string SavePath;
string SaveRGBPath;
string SavedepthPath;
string SaveImuPath;

void SaveRGB(const sensor_msgs::ImageConstPtr &rgb);
void SaveDepth( const sensor_msgs::ImageConstPtr &depth);
void SaveIMU(const sensor_msgs::Imu& imudata);
void Imu_callback(const sensor_msgs::ImuConstPtr &acc, const sensor_msgs::ImuConstPtr &angle);

int main(int argc,char** argv)
{
    ros::init(argc,argv,"SaveData");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;
    string color_topic,depth_topic,accel_topic,gyro_topic,imu_topic,left_topic,right_topic;

    nh.param<string>("color_topic",color_topic,"/camera/color/image_raw");
    nh.param<string>("depth_topic",depth_topic,"/camera/depth/image_rect_raw");
    nh.param<string>("left_topic",left_topic,"/camera/infra1/image_rect_raw");
    nh.param<string>("right_topic",right_topic,"/camera/infra2/image_rect_raw");
    nh.param<string>("accel_topic",accel_topic,"/camera/accel/sample");
    nh.param<string>("gyro_topic",gyro_topic,"/camera/gyro/sample");
    nh.param<string>("imu_topic",imu_topic,"/camera/imu");
    nh.param<string>("SavePath",SavePath,"/home/wanggong/catkin_ws/src/save_image_imu/data/");
    cout<<SavePath.c_str()<<endl;
    SaveRGBPath = SavePath + "rgb/";
    SavedepthPath = SavePath + "depth/";
    SaveImuPath = SavePath + "imu/";

    string image_txt = SavePath+"rgb.txt";
    string depth_txt = SavePath+"depth.txt";
    string imu_txt = SavePath +"imu.txt";
    cout<<depth_txt.c_str()<<endl;
    rgbf.open(image_txt.c_str());
    depthf.open(depth_txt.c_str());
    imuf.open(imu_txt.c_str());
    ostringstream mos_imu;
    mos_imu<<fixed<<"stamptime,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z";
    imuf<<mos_imu.str()<<endl;


    ros::Subscriber rgb_sub = nh.subscribe(color_topic,100,&SaveRGB);
    ros::Subscriber depth_sub = nh.subscribe(depth_topic,100,&SaveDepth);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic,1000,&SaveIMU);

    message_filters::Subscriber<sensor_msgs::Imu> acc_sub(nh,accel_topic,1000);
    message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(nh,gyro_topic,1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10),acc_sub,gyro_sub);
    sync.registerCallback(boost::bind(&Imu_callback,_1,_2));


    ros::spin();
    ros::waitForShutdown();
    rgbf.close();
    depthf.close();
    imuf.close();
    return 0;
}


void SaveRGB(const sensor_msgs::ImageConstPtr &rgb)
{
    cv::Mat image;
    cv_bridge::CvImageConstPtr cvimage;
    cvimage = cv_bridge::toCvShare(rgb,"bgr8");
    cvimage->image.copyTo(image);
    ostringstream mos_rgd;
    mos_rgd<<fixed<<rgb->header.stamp;
    string rgb_name = SaveRGBPath + mos_rgd.str()+".png";
    cout<<rgb_name.c_str()<<endl;
    rgbf<<mos_rgd.str()<<" "<<"/rgb/"<<mos_rgd.str()<<".png\n";
    cv::imwrite(rgb_name,image);
}

void SaveDepth( const sensor_msgs::ImageConstPtr &depth)
{
    cv::Mat image;
    cv_bridge::CvImageConstPtr cvimage;
    cvimage = cv_bridge::toCvShare(depth,"16UC1");
    cvimage->image.copyTo(image);
    ostringstream mos_depth;
    mos_depth<<fixed<<depth->header.stamp;
    string depth_name = SavedepthPath + mos_depth.str()+".png";
    depthf<<mos_depth.str()<<" "<<"/depth/"<<mos_depth.str()<<".png\n";
    cv::imwrite(depth_name,image);
}

void SaveIMU(const sensor_msgs::Imu& imudata)
{
    ostringstream mos_imu;
    mos_imu<<fixed<<imudata.header.stamp<<" "
    <<float(imudata.linear_acceleration.x)<<" "
    <<float(imudata.linear_acceleration.y)<<" "
    <<float(imudata.linear_acceleration.z)<<" "
    <<float(imudata.angular_velocity.x)<<" "
    <<float(imudata.angular_velocity.y)<<" "
    <<float (imudata.angular_velocity.z);
    imuf<<mos_imu.str()<<endl;
}

void Imu_callback(const sensor_msgs::ImuConstPtr &acc, const sensor_msgs::ImuConstPtr &angle)
{
    ostringstream mos_imu;
    mos_imu<<fixed<<acc->header.stamp<<" "
    <<acc->linear_acceleration.x<<" "
    <<acc->linear_acceleration.y<<" "
    <<acc->linear_acceleration.z<<" "
    <<angle->angular_velocity.x<< " "
    <<angle->angular_velocity.y<< " "
    <<angle->angular_velocity.z;
    imuf<<mos_imu.str()<<endl;
}