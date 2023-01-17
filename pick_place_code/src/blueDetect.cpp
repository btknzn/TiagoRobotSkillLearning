// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
	

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>


#include <string> 
#include <batu_training/objectLocation.h>


typedef pcl::PointXYZ PointT;

class VisionDetect
{
public:
	VisionDetect(ros::NodeHandle nh_);
	batu_training::objectLocation msg;

protected:
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	ros::Subscriber sub;
	ros::Publisher pub;

};


VisionDetect::VisionDetect(ros::NodeHandle nh_)
{
	sub = nh_.subscribe("/xtion/depth_registered/points", 1, &VisionDetect::cloud_cb, this);
	pub = nh_.advertise<batu_training::objectLocation>("blueArea", 1);
}

void VisionDetect::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*input, *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
	voxel.setInputCloud (cloud);
	voxel.setLeafSize (0.01f, 0.01f, 0.01f);
	voxel.filter (*cloud_voxel);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud_voxel);
  	sor.setMeanK (50);
  	sor.setStddevMulThresh (0.2);
  	sor.filter (*cloud_sor);



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_area_filtered1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_area_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_area_filtered3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass1;
	pass1.setInputCloud (cloud_sor);
	pass1.setFilterFieldName ("x");
	pass1.setFilterLimits (-1.0, 1.0);
	pass1.filter (*cloud_area_filtered1);
	pcl::PassThrough<pcl::PointXYZRGB> pass2;
	pass2.setInputCloud (cloud_area_filtered1);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (0, 2);
	pass2.filter (*cloud_area_filtered2);
	pcl::PassThrough<pcl::PointXYZRGB> pass3;
	pass3.setInputCloud (cloud_area_filtered2);
	pass3.setFilterFieldName ("y");
	pass3.setFilterLimits (0, 2.0);
	pass3.filter (*cloud_area_filtered3);



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter_table(new pcl::PointCloud<pcl::PointXYZRGB>);




	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	std::uint32_t rgb_filter1 = ((std::uint32_t)0<<16  | (std::uint32_t)0<<8  | (std::uint32_t)150);
	std::uint32_t rgb_filter2 = ((std::uint32_t)20<<0  | (std::uint32_t)0<<8 | (std::uint32_t)255);

	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("rgb", pcl::ComparisonOps::GE, *reinterpret_cast<float*>(&rgb_filter1))));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("rgb", pcl::ComparisonOps::LE, *reinterpret_cast<float*>(&rgb_filter2))));
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud_area_filtered3);
	condrem.setKeepOrganized(true);
	condrem.filter(*cloud_filter_table);
	float x_mean;
	float y_mean;
	float z_mean;
	x_mean = 0;
	y_mean = 0;

	z_mean = 0;
	int counter = 0;


	for (const auto& point: *cloud_filter_table){
		if (isnan(point.x) && isnan(point.y) &&isnan(point.z) ){
			continue;
		}
		counter = counter+1;
		x_mean += point.x;
		y_mean += point.y;
		z_mean += point.z; 

	}

	std::cout << x_mean << std::endl;
	msg.x = x_mean/counter;
	msg.y = y_mean/counter;
	msg.z = z_mean/counter;
	std::string f_str_x = std::to_string (msg.x);
	std::string f_str_y = std::to_string (msg.y);
	std::string f_str_z = std::to_string (msg.z);

	printf(" X value");
	std::cout<< f_str_x << " \n";
	printf(" Y value");
	std::cout<< f_str_y << " \n";
	printf(" Z value");
	std::cout<< f_str_z << " \n";



	pcl::io::savePCDFileASCII ("wholescene_blue.pcd", *cloud);
	pcl::io::savePCDFileASCII ("table_blue.pcd", *cloud_filter_table);

	pub.publish(msg);
}




int main(int argc, char** argv)
{
    ROS_INFO("Working fine");
	ros::init(argc, argv, "blueArea");
	ros::NodeHandle nh;
	VisionDetect ts(nh);
	ros::spin();
}	