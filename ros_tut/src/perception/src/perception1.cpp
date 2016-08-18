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
	float filter_voxel_size;//4
	bool filter_voxel;

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
public:

  MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments )
  {
		po_topic_name= arguments[1];
		filter_radius= atof (arguments[2]);
		filter_neighbour= atoi (arguments[3]);
		filter_voxel_size= atof (arguments[4]);
		filter_voxel=false;
		if(filter_voxel_size>0)
			filter_voxel=true;

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered=cloud.makeShared();

	// voxel filtering
	if(filter_voxel) {
		pcl::VoxelGrid<pcl::PointXYZ >sor;
		sor.setInputCloud (cloud_filtered);
		sor.setLeafSize (filter_voxel_size, filter_voxel_size, filter_voxel_size);
		sor.filter (*cloud_filtered);
	}

	// radius filtering
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud (cloud_filtered);
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
