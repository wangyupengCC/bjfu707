#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "MapLocation.h"

using namespace std;
using namespace cv;
class MapLocation;
int main(int argc,char** argv)
{
    cv::FileStorage fs;
    string settingFile = string (argv[1]);
    fs.open(settingFile.c_str(),cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Cloud not open setting file" <<settingFile<<endl;
    }
    MapLocation* mapLocation = new MapLocation(fs);
    return 0;
}


