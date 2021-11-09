#include<iostream>
#include<chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "Converter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_rgb,pub_tcw;
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM)
    {
        pub_rgb=nh.advertise<sensor_msgs::Image>("KeyFrame/RGB",10);
        pub_tcw= nh.advertise<nav_msgs::Odometry> ("KeyFrame/Tcw", 10);
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    ORB_SLAM2::Converter converter;
    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,false);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 15);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 15);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();
    SLAM.Shutdown();
    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat Tcw;
    bool isKeyFrame;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),isKeyFrame,false);
    sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
    Eigen::Isometry3d Tcw_ = converter.toSE3Quat(Tcw);
    Eigen::Isometry3d Twc =Tcw_.inverse();
    Eigen::Quaterniond Qwc = Eigen::Quaterniond(Twc.rotation());
    Eigen::Vector3d Pwc = Twc.translation();
    nav_msgs::Odometry odometry;
    odometry.header.frame_id ="map";
    odometry.header.stamp = rgb_msg->header.stamp;
    odometry.pose.pose.position.x = Pwc.x();
    odometry.pose.pose.position.y = Pwc.y();
    odometry.pose.pose.position.z = Pwc.z();
    odometry.pose.pose.orientation.x = Qwc.x();
    odometry.pose.pose.orientation.y = Qwc.y();
    odometry.pose.pose.orientation.z = Qwc.z();
    odometry.pose.pose.orientation.w = Qwc.w();
    pub_tcw.publish(odometry);
    pub_rgb.publish(rgb_msg);
}

