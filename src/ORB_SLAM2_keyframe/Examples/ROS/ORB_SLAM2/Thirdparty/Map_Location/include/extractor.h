#ifndef MY_SLAM_EXTRACTOR_H
#define MY_SLAM_EXTRACTOR_H

#include <list>
#include <opencv/cv.h>
#include <vector>

namespace My_SLAM
{
    // 分配四叉树时用到的结点类型
    class ExtractorNode
    {
    public:
        ExtractorNode():bNoMore(false){}
        void DivideNode(ExtractorNode &n1,ExtractorNode &n2,ExtractorNode &n3,ExtractorNode &n4);
        ///保存有当前节点的特征点
        std::vector<cv::KeyPoint> vKeys;
        ///当前节点所对应的图像坐标边界
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        ///如果节点中只有一个特征点的话，说明这个节点不能够再进行分裂了，这个标志置位
        ///这个节点中如果没有特征点的话，这个节点就直接被删除了
        bool bNoMore;
    };
    ///*************ORB特征点提取器**************/
    class ORBextractor
    {
    public:
        enum {
            HARRIS_SCORE=0,
            FAST_SCORE=1
        };
        ORBextractor(int nfeatures,float scaleFactor,int nlevels,int iniThFAST,int minThFAST);
        ~ORBextractor(){}
///       nfeatures         指定要提取出来的特征点数目
///       scaleFactor       图像金字塔的缩放系数
///       nlevels           指定需要提取特征点的图像金字塔层
///       iniThFAST         初始的默认FAST响应值阈值
///       minThFAST         较小的FAST响应值阈值

        void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,cv::OutputArray descriptors);
///        image         要操作的图像
///        mask          图像掩膜，辅助进行图片处理，可以参考[https://www.cnblogs.com/skyfsm/p/6894685.html
///        keypoints    保存提取出来的特征点的向量
///        descriptors  输出用的保存特征点描述子的cv::Mat

        int inline GetLevels(){
            return nlevels;}
        float inline GetScaleFactor(){
            return scaleFactor;}
        ///获取当前提取器所在的图像的缩放因子，这个不带s的因子表示是相临近层之间的
        std::vector<float> inline GetScaleFactors(){
            return mvScaleFactor;}
        ///获取图像金字塔中每个图层相对于底层图像的缩放因子
        std::vector<float> inline GetInverseScaleFactors(){
            return mvInvScaleFactor;}
        ///获取上面的那个缩放因子s的倒数
        std::vector<float> inline GetScaleSigmaSquares(){
            return mvLevelSigma2;}
        ///每层图像相对于初始图像缩放因子的平方，参考cpp文件中类构造函数的操作
        std::vector<float> inline GetInverseScaleSigmaSquares(){
            return mvInvLevelSigma2;}
         ///获取上面sigma平方的倒数
        std::vector<cv::Mat> mvImagePyramid;
        ///这个是用来存储图像金字塔的变量，一个元素存储一层图像

    protected:
        void ComputePyramid(cv::Mat iamge);
        ///针对给出的一张图像，计算其图像金字塔

        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
        ///以八叉树分配特征点的方式，计算图像金字塔中的特征点
        ///这里两层vector的意思是，第一层存储的是某张图片中的所有特征点，而第二层则是存储图像金字塔中所有图像的vectors

        std::vector<cv::KeyPoint> DistributeOctTree(
                const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY,
                const int &nFeatures, const int &level);
        /**
         * @brief 对于某一图层，分配其特征点，通过八叉树的方式
         * @param[in] vToDistributeKeys         等待分配的特征点
         * @param[in] minX                      分发的图像范围
         * @param[in] maxX                      分发的图像范围
         * @param[in] minY                      分发的图像范围
         * @param[in] maxY                      分发的图像范围
         * @param[in] nFeatures                 设定的、本图层中想要提取的特征点数目
         * @param[in] level                     要提取的图像所在的金字塔层
         * @return std::vector<cv::KeyPoint>
         */
        ///这是使用另外一种老办法提取并平均特征点的方法，但是在实际的程序中并没有用到
        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
        std::vector<cv::Point> pattern;             ///<用于计算描述子的随机采样点集合

        int nfeatures;			                    ///<整个图像金字塔中，要提取的特征点数目
        double scaleFactor;		                    ///<图像金字塔层与层之间的缩放因子
        int nlevels;			                    ///<图像金字塔的层数
        int iniThFAST;			                    ///<初始的FAST响应值阈值
        int minThFAST;			                    ///<最小的FAST响应值阈值

        std::vector<int> mnFeaturesPerLevel;		///<分配到每层图像中，要提取的特征点数目

        std::vector<int> umax;	                    ///<计算特征点方向的时候，有个圆形的图像区域，这个vector中存储了每行u轴的边界（四分之一，其他部分通过对称获得）

        std::vector<float> mvScaleFactor;		    ///<每层图像的缩放因子
        std::vector<float> mvInvScaleFactor;        ///<以及每层缩放因子的倒数
        std::vector<float> mvLevelSigma2;		    ///<存储每层的sigma^2,即上面每层图像相对于底层图像缩放倍数的平方
        std::vector<float> mvInvLevelSigma2;	    ///<sigma平方的倒数
    };
}

#endif //MY_SLAM_EXTRACTOR_H
