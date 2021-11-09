#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "../include/save_rgbd.h"
using namespace std;
using namespace cv;
class save_rgbd_node;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"SAVE_RGBD");
    if(!ros::ok())
        return 0;
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    string rgb_topic,depth_topic,Tcw_topic,SaveDataPath;
    nh.param<string>("rgb_topic",rgb_topic,"/KeyFrame/RGB");
    nh.param<string>("depth_topic",depth_topic,"/KeyFrame/Depth");
    nh.param<string>("Tcw_topic",Tcw_topic,"/KeyFrame/Tcw");
    nh.param<string>("SaveDataPath",SaveDataPath,"/home/wanggong/catkin_ws/src/save_rgbd/data/data1/");
    save_rgbd_node *mSave = new save_rgbd_node(SaveDataPath);

    message_filters::Subscriber<sensor_msgs::Image> rgbSub(n,rgb_topic,15);
    message_filters::Subscriber<sensor_msgs::Image> depthSub(n,depth_topic,15);
    message_filters::Subscriber<nav_msgs::Odometry> Tcw_sub(n,Tcw_topic,15);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10),rgbSub,depthSub,Tcw_sub);
    sync.registerCallback(boost::bind(&save_rgbd_node::callback,mSave,_1,_2,_3));

    ros::spin();
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}