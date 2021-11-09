#ifndef _SAVE_RGBD_H
#define _SAVE_RGBD_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <thread>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
using namespace cv;
class save_rgbd_node
{
private:
    string SaveDepthPath;
    string SaveRGBPath;
    string SaveTcwPath;
    string ImageTxt;
    string DepthTxt;
    string KeyFrameTrajectoryTUM;
    cv::Mat mrgb;
    cv::Mat mdepth;
    nav_msgs::OdometryConstPtr mTcw;
    struct timeval mtimeval;
    struct timezone mtz;



public:
    save_rgbd_node(string &SavePath);
    ~save_rgbd_node();
    ofstream rgbf;
    ofstream depthf;
    ofstream trajf;

    void SaveKeyFrameRGB();
    void SaveKeyFrameDepth();
    void SaveKeyFrameTrajectory();
    void callback(const sensor_msgs::ImageConstPtr& rgdImage,const sensor_msgs::ImageConstPtr& depthImage,const nav_msgs::OdometryConstPtr& Tcw);

};





#endif