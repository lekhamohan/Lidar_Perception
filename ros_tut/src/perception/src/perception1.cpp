#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//intensity data getting lost

	//(*input).fields[3].name = "intensities";
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  //pcl::PointCloud<pcl::PointXYZ> cloud1;
	std::vector< int > a;
	pcl::removeNaNFromPointCloud (cloud,cloud,a);
	std::cout<<"1\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	//cloudPtr=pcl::PointCloud<pcl::PointXYZ>::Ptr(&cloud);
	cloudPtr=cloud.makeShared();


	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// build the filter
	outrem.setInputCloud(cloudPtr);
	outrem.setRadiusSearch(0.05);
	outrem.setMinNeighborsInRadius (10);
	//outrem.setKeepOrganized(false);
	// apply filter
	outrem.filter (*cloud_filtered);


  //pcl::PCLPointCloud2 outpcl;
	sensor_msgs::PointCloud2 output;
  //pcl_conversions::toROSMsg(cloud, output);
	pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, output);

  // Publish the data
	//for(int i=1; i<4;i++) 
		//std::cout<<	output.fields[i].name<<"\n";
	//output.fields[3].name = "intensities";
  pub.publish (output);
//cloudPtr=NULL;
//delete cloudPtr;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
