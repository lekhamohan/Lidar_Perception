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

void PcltoROSMSG(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, sensor_msgs::PointCloud2& rosMsg) {
	//function to convert Poincloud<pointXYZ> to pointcloud2 for some reason the inbuild code doesnt work

  //pcl::PCLPointCloud2 outpcl;
	//sensor_msgs::PointCloud2 output;
  //pcl_conversions::toROSMsg(cloud, output);
	pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pclCloud, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, rosMsg);
}


/////////////////////////////////////////////////////////////

class MySubscriber
{
  std::string po_topic_name;//1
  ros::Subscriber po_sub;
	float filter_radius;//2
	int filter_neighbour;//3

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
public:

  MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments )
  {
		po_topic_name= arguments[1];
		filter_radius= atof (arguments[2]);
		filter_neighbour= atoi (arguments[3]);
    po_sub = ao_nh.subscribe (po_topic_name, 1, &MySubscriber::cloud_cb, this );
		pub = ao_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  }
};

//////////////////////////////////////////////

void MySubscriber::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//intensity data getting lost
	
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
 	std::vector< int > a;
	pcl::removeNaNFromPointCloud (cloud,cloud,a);
	//std::cout<<"1\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	cloudPtr=cloud.makeShared();

	// filtering
	//std::cout<<"radius "<<filter_radius<<std::endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	outrem.setInputCloud (cloudPtr);
	outrem.setRadiusSearch (filter_radius);
	outrem.setMinNeighborsInRadius (filter_neighbour);
	outrem.filter (*cloud_filtered);

  // Publish the data
	sensor_msgs::PointCloud2 output;
	PcltoROSMSG(cloud_filtered,output);
  pub.publish (output);
}

/////////////////////////////////////////////////////

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

	MySubscriber s(nh,argc, argv);

  // Spin
  ros::spin ();
}
