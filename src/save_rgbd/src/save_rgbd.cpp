#include "../include/save_rgbd.h"
#include <thread>
#include <time.h>
using namespace std;
using namespace cv;
save_rgbd_node::save_rgbd_node(string &SaveDataPath)
{
    this->ImageTxt = SaveDataPath + "rgb.txt";
    this->SaveRGBPath = SaveDataPath +"rgb/";
    this->DepthTxt = SaveDataPath +"depth.txt";
    this->SaveDepthPath = SaveDataPath + "depth/";
    this->KeyFrameTrajectoryTUM = SaveDataPath + "KeyFrameTrajectory.txt";
    rgbf.open(ImageTxt.c_str());
    depthf.open(DepthTxt.c_str());
    trajf.open(KeyFrameTrajectoryTUM.c_str());
}

save_rgbd_node::~save_rgbd_node()
{
    rgbf.close();
    depthf.close();
    trajf.close();
}

void save_rgbd_node::SaveKeyFrameRGB()
{
   rgbf<<fixed;
   ostringstream mos_rgd;
    mos_rgd<<mtimeval.tv_sec<<"."<<mtimeval.tv_usec;
    string rgb_name = SaveRGBPath + mos_rgd.str()+".png";
    rgbf<<mos_rgd.str()<<" "<<"/rgb/"<<mos_rgd.str()<<".png"<<endl;
    imwrite(rgb_name,mrgb);
}

void save_rgbd_node::SaveKeyFrameDepth()
{
    depthf<<fixed;
    ostringstream mos_depth;
    mos_depth<<mtimeval.tv_sec<<"."<<mtimeval.tv_usec;
    string depth_name = SaveDepthPath + mos_depth.str()+".png";
    depthf<<mos_depth.str()<<" "<<"/depth/"<<mos_depth.str()<<".png"<<endl;;
    imwrite(depth_name,mdepth);
}

void save_rgbd_node::SaveKeyFrameTrajectory()
{
    ostringstream mos_traj;
    trajf<<fixed;
    mos_traj<<mtimeval.tv_sec<<"."<<mtimeval.tv_usec<<" "<<float(mTcw->pose.pose.position.x)<<" "<<float(mTcw->pose.pose.position.y)<<" "<<float(mTcw->pose.pose.position.z)<<" "<<float(mTcw->pose.pose.orientation.x)
    <<" "<<float(mTcw->pose.pose.orientation.y)<<" "<<float (mTcw->pose.pose.orientation.z)<<" "<<float (mTcw->pose.pose.orientation.w);
    trajf<<mos_traj.str()<<endl;
}

void save_rgbd_node::callback(const sensor_msgs::ImageConstPtr &rgdImage, const sensor_msgs::ImageConstPtr &depthImage,
                              const nav_msgs::OdometryConstPtr &Tcw) {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(rgdImage, "bgr8");
    pCvImage->image.copyTo(mrgb);
    pCvImage = cv_bridge::toCvShare(depthImage, "16UC1");
    pCvImage->image.copyTo(mdepth);
    gettimeofday(&mtimeval,&mtz);
    mTcw =Tcw;
    SaveKeyFrameRGB();
    SaveKeyFrameDepth();
    SaveKeyFrameTrajectory();
}
