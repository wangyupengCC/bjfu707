#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>				// 法向量估计
#include <pcl/segmentation/sac_segmentation.h>	// 模型分割
#include <pcl/filters/extract_indices.h>		// 索引提取
#include <pcl/visualization/cloud_viewer.h>		// 可视化

using namespace std;

typedef pcl::PointXYZRGBA PointT;

int main(int argc,char** argv)
{	
	//-----------------------------加载点云--------------------------------
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile(string(argv[1]), *cloud) < 0)
	{
		PCL_ERROR("\a点云文件不存在！\n");
		system("pause");
		return -1;
	}
	cout << "->加载数据点的个数：" << cloud->points.size() << endl;
	//=====================================================================

	//-----------------------------法线估计--------------------------------
	cout << "->正在计算法线..." << endl;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;	// 创建法向量估计对象
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);						// 设置搜索方式
	ne.setInputCloud(cloud);						// 设置输入点云
	ne.setKSearch(50);								// 设置K近邻搜索点的个数
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*cloud_normals);						// 计算法向量，并将结果保存到cloud_normals中
	//----------------------------圆柱体分割--------------------------------
	cout << "->正在圆柱体分割..." << endl;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;		// 创建圆柱体分割对象
	seg.setInputCloud(cloud);										// 设置输入点云：待分割点云
	seg.setOptimizeCoefficients(true);								// 设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);						// 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);								// 设置采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.1);								// 设置表面法线权重系数
	seg.setMaxIterations(10);									// 设置迭代的最大次数
	seg.setDistanceThreshold(0.05);									// 设置内点到模型距离的最大值
	seg.setRadiusLimits(3.0, 4.0);									// 设置圆柱模型半径的范围
	seg.setInputNormals(cloud_normals);								// 设置输入法向量
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);	// 保存分割结果

	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);	// 保存圆柱体模型系数
	seg.segment(*inliers_cylinder, *coefficients_cylinder);			// 执行分割，将分割结果的索引保存到inliers_cylinder中，同时存储模型系数coefficients_cylinder
	cout << "\n\t\t-----圆柱体系数-----" << endl;
	cout << "轴线一点坐标：(" << coefficients_cylinder->values[0] << ", "
		<< coefficients_cylinder->values[1] << ", "
		<< coefficients_cylinder->values[2] << ")"
		<< endl;
	cout << "轴线方向向量：(" << coefficients_cylinder->values[3] << ", "
		<< coefficients_cylinder->values[4] << ", "
		<< coefficients_cylinder->values[5] << ")"
		<< endl;
	cout << "圆柱体半径：" << coefficients_cylinder->values[6] << endl;
	//=====================================================================

	//------------------------------提取分割结果----------------------------
	cout << "->正在提取分割结果..." << endl;
	pcl::ExtractIndices<PointT> extract;	// 创建索引提取点对象
	extract.setInputCloud(cloud);			// 设置输入点云：待分割点云
	extract.setIndices(inliers_cylinder);	// 设置内点索引
	extract.setNegative(false);				// 默认false，提取圆柱体内点；true，提取圆柱体外点
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);		// 执行滤波，并将结果点云保存到cloud_cylinder中
	//=====================================================================

	//----------------------------------保存分割结果------------------------
	if (!cloud_cylinder->points.empty())
	{
		pcl::PCDWriter writer;
		writer.write("cylinder.pcd", *cloud_cylinder, true);
		cout << "->圆柱体模型点云个数：" << cloud_cylinder->size() << endl;
	}
	else
	{
		PCL_ERROR("未提取出圆柱体模型点！\a\n");
	}
	//=====================================================================

	//-------------------------可视化分割结果（可选操作）--------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("可视化分割结果"));

	///视口1：原始点云
	int v1;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); //设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("original_point_cloud", 10, 10, "v1 text", v1);
	viewer->addPointCloud<PointT>(cloud, "original_point_cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "original_point_cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_point_cloud", v1);

	///视口2：分割后点云
	int v2;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("segment_point_cloud", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> set_color(cloud_cylinder, 0, 255, 0);
	viewer->addPointCloud<PointT>(cloud_cylinder, set_color, "segment_point_cloud", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment_point_cloud", v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//=====================================================================

	return 0;
}
