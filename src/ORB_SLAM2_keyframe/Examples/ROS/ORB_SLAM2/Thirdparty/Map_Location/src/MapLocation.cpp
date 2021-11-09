#include "MapLocation.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "DBoW3/DBoW3.h"
using namespace std;
using namespace cv;
using namespace My_SLAM;

double findMax(vector<double> vec) {
    double max = -999.0;
    for (auto v : vec) {
        if (max < v) max = v;
    }
    return max;
}

std::string DoubleToString(const double value, unsigned int precision)
{
    std::ostringstream out;
    if (precision > 0)
        out.precision(precision);

    out << value;
    return out.str();
}

int getPositionOfMax(vector<double> vec, double max) {
    auto distance = find(vec.begin(), vec.end(), max);
    return distance - vec.begin();
}

MapLocation::MapLocation(cv::FileStorage& fs) {
    int nFeatures = fs["ORBextractor.nFeatures"];
    float fScaleFactor = fs["ORBextractor.scaleFactor"];
    int nLevels = fs["ORBextractor.nLevels"];
    int fIniThFAST = fs["ORBextractor.iniThFAST"];
    int fMinThFAST = fs["ORBextractor.minThFAST"];
    mORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    string KeyFramePath = string (fs["KeyFramePath"]) + "/rgb.txt";
    LoadImages(KeyFramePath,mKeyFrameNames,mKeyFrameStamps);
    mKeyFrameImages.resize(mKeyFrameNames.size());
    for (int i = 0; i < mKeyFrameNames.size();i++) {
        mKeyFrameNames[i] = "/home/wanggong/catkin_ws/src/save_rgbd/data/data1/" + mKeyFrameNames[i];
        mKeyFrameImages[i] = imread(mKeyFrameNames[i], CV_LOAD_IMAGE_COLOR);
    }
    mKeyFrameImages_Gray.resize(mKeyFrameImages.size());
    rgb_to_Gray(mKeyFrameImages,mKeyFrameImages_Gray);
    mVocabularyPath = string (fs["FilePaths"]["VocabPath"]);
    int nImages = mKeyFrameImages_Gray.size();
    for (int i = 0; i < nImages; ++i) {
        (*mORBextractor)(mKeyFrameImages_Gray[i],cv::Mat(),mKFkeypoints,mKFdescriptor);
        mKFs_dedescriptors.push_back(mKFdescriptor);
    }
    std::cout<<"Train Done!"<<std::endl;
}

MapLocation::~MapLocation()
{

}

void MapLocation::rgb_to_Gray(vector<cv::Mat> &KeyFrame, vector<cv::Mat> &KeyFrame_Gray) {
    int n = KeyFrame.size();
    for (int i = 0; i <n-1;i++) {
        if(KeyFrame[i].channels()==3)
        {
            cvtColor(KeyFrame[i],KeyFrame_Gray[i],CV_RGB2GRAY);
        }
        else if(KeyFrame[i].channels()==4)
        {
            cvtColor(KeyFrame[i],KeyFrame_Gray[i],CV_RGBA2GRAY);
        }
    }
}

int MapLocation::FindMatchImage(cv::Mat &Frame) {
    vector<KeyPoint> keypoints;
    DBoW3::Vocabulary vocabulary(string(mVocabularyPath)+"/"+"Vocabulary.yml.gz");
    DBoW3::BowVector v1,v2;
    cv::Mat fdescriptor;
    (*mORBextractor)(Frame,cv::Mat(),keypoints,fdescriptor);
    vocabulary.transform(fdescriptor,v2);
    vector<double> Score(mKFs_dedescriptors.size());
    for (int i=0;i<mKFs_dedescriptors.size();i++)
    {

        vocabulary.transform(mKFs_dedescriptors[i],v1);
        double score = vocabulary.score(v1,v2);
        Score[i] = score;
    }
    double MaxScore = findMax(Score);
    int Number = getPositionOfMax(Score,MaxScore);
    cout<<"best match"<<Number<<endl;
    return Number;
}
void MapLocation::GetKeyFrameTrajectory(cv::Mat &CurrentKeyFrame, Eigen::Vector3d pwc,
                                        Eigen::Quaterniond quaterniond) {
    cv::Mat CurrentKeyFrame_Gray;
    rgb_to_Gray(CurrentKeyFrame,CurrentKeyFrame_Gray);
    int N = FindMatchImage(CurrentKeyFrame_Gray);
    string key = DoubleToString(mKeyFrameStamps[N],16);
    ifstream f;
    f.open("/home/wanggong/catkin_ws/src/save_rgbd/data/data1/KeyFrameTrajectory.txt");
    int linenum = 0;
    float data[7] = {0,0,0,0,0,0,0};
    if (!f.eof())
    {
        string line;
        stringstream ss;
        getline(f,line);
        if (line.find(key) != string::npos)
        {
            ss<<line;
            ss>>line;
            for (int i = 0; i < 7; i++) {
                ss >> data[i];
            }
            Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
            quaterniond = q;
            pwc.x() = data[0];
            pwc.y() = data[1];
            pwc.z() = data[2];
        }
    }
}

void MapLocation::rgb_to_Gray(cv::Mat &Img, cv::Mat &GrayImg) {
    if(Img.channels()==3)
    {
        cvtColor(Img,GrayImg,CV_RGB2GRAY);
    }
    else if(Img.channels()==4)
    {
        cvtColor(Img,GrayImg,CV_RGBA2GRAY);
    }
}