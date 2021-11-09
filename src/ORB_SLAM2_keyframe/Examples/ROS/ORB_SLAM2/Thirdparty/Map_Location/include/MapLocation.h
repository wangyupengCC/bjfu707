#ifndef MAP_LOCATION_MAPLOCATION_H
#define MAP_LOCATION_MAPLOCATION_H

#include "extractor.h"
#include "LoadImages.h"
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <mutex>
#include <thread>
#include <DBoW3/DBoW3.h>
class MapLocation
{
public:
    MapLocation(cv::FileStorage& fs);
    ~MapLocation();
    void GetKeyFrameTrajectory(cv::Mat& CurrentKeyFrame_Gary,Eigen::Vector3d pwc,Eigen::Quaterniond quaterniond);
    void rgb_to_Gray(cv::Mat& Img,cv::Mat& GrayImg);
    void rgb_to_Gray(vector<cv::Mat>& KeyFrame,vector<cv::Mat>& KeyFrame_Gray);
    int FindMatchImage(cv::Mat &Frame);
    bool mReFlag = false;
    ///***********KeyFrame**********///
    vector<string> mKeyFrameNames;
    vector<double> mKeyFrameStamps;
    vector<cv::Mat> mKeyFrameImages;
    vector<cv::Mat> mKeyFrameImages_Gray;
    cv::Mat mKFdescriptor;
    vector<cv::KeyPoint> mKFkeypoints;
    vector<cv::Mat> mKFs_dedescriptors;
///***********KeyFrameTcw**********///
///***********CurrentKeyFrame**********///

    cv::Mat mCurrentKeyFrame;
///***********CurrentKeyFrame**********///

///***********Vocabulary**********///
    string mVocabularyPath;
    vector<double> mStamped;
    My_SLAM::ORBextractor* mORBextractor;
};

#endif //MAP_LOCATION_MAPLOCATION_H
