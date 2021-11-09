#include <iostream>
#include <ros/ros.h>  
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;

int main (int argc, char **argv)  
{  
	std::string topic,path,frame_id;
    PointCloud::Ptr cloud(new PointCloud);

    sensor_msgs::PointCloud2 output;
    int hz;

	ros::init (argc, argv, "publish_pointcloud");  
	ros::NodeHandle nh;  
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("path", path, "/home/wanggong/catkin_ws/src/publish_pointcloud/data/Map.pcd");
	nh_private.param<std::string>("frame_id", frame_id, "world");
	nh_private.param<std::string>("topic", topic,"Point_Cloud_Map");
    nh_private.param<int>("hz", hz, 50);
    pcl::visualization::CloudViewer viewer("pointcloudmap");

    tf::TransformBroadcaster broadcaster;
    ros::Publisher _octomap3D_pub = nh.advertise<octomap_msgs::Octomap>("/octo3Dmap",50);
    ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("PointCloud",50);
    pcl::io::loadPCDFile (path, *cloud);
    PointCloud transformed_cloud;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, 0.0;
    transform.rotate (Eigen::AngleAxisf (-(M_PI/2), Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloud, transformed_cloud, transform);
    pcl::toROSMsg(transformed_cloud,output);

    /***************RANSAC拟合平面*****************/
//    //创建一个模型参数对象，用于记录结果
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//     //inliers表示误差能容忍的点 记录的是点云的序号
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//     // 创建一个分割器
//     pcl::SACSegmentation<PointType> seg;
//     // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
//     seg.setOptimizeCoefficients (true);
//     // Mandatory-设置目标几何形状
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     //分割方法：随机采样法
//     seg.setMethodType (pcl::SAC_RANSAC);
//     //设置误差容忍范围，也就是我说过的阈值
//     seg.setDistanceThreshold (0.01);
//     //输入点云
//     seg.setInputCloud (cloud);
//     //分割点云
//     seg.segment (*inliers, *coefficients);
   
    /***************RANSAC拟合平面*****************/

    octomap::OcTree tree(0.3);
    octomap::Pointcloud octo_point;

    for(auto p:transformed_cloud.points)
    {   
        tree.updateNode(octomap::point3d(p.x,p.y,p.z),true);
    }
    tree.updateInnerOccupancy();
    octomap_msgs::Octomap octomap_fullmsg;
    octomap_msgs::fullMapToMsg(tree, octomap_fullmsg);
    octomap_fullmsg.header.frame_id = "map";
    output.header.frame_id = "map";
    ros::Rate loop_rate(50);
    while (ros::ok())
    {   
        viewer.showCloud(cloud);
        octomap_fullmsg.header.stamp = ros::Time::now();
        output.header.stamp = ros::Time::now();
        _octomap3D_pub.publish(octomap_fullmsg);
        point_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;  
}






