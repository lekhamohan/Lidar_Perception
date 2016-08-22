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

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <unistd.h>
#include <iostream>
#include <pcl/io/io.h>

ros::Publisher pub;
//new_branch

void PcltoROSMSG(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, sensor_msgs::PointCloud2& rosMsg) {
  //function to convert Poincloud<pointXYZ> to pointcloud2 for some reason the inbuild code doesnt work
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pclCloud, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, rosMsg);
}


////////////////////////////////////////////////////////////////////////

class MySubscriber
{
  std::string po_topic_name;//1
  ros::Subscriber po_sub;
  float filter_radius;//2
  int filter_neighbour;//3
    
  bool filter_voxel;
  
  float filter_voxel_size;//4
  
  bool do_region_flow_cluster;
  
  int cluster_min_size;//5 = 50;
  int cluster_max_size;//6 =100;
  float normal_search_radius;//7 =.03;
  int normal_search_neigh;//8 =50;
  float SmoothnessThreshold;//9 =3.0;
  float CurvatureThreshold;//10 =1.0;
  int reg_Neighbours;//11 =30;  
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_final;

  float Ransac_dist_thres;
  bool updated;

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
  void region_flow_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);//, pcl::visualization::CloudViewer& viewer);
  void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
public:

  MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments );
  
};

////////////////////////////////////////////////////////////////////////
int num_arguments= 6;
MySubscriber::MySubscriber( ros::NodeHandle ao_nh, int no_arg, char** arguments )
{
  cout<<"num of arg "<<no_arg<<"\n";
  if(no_arg< num_arguments) {
    cout<<"\n usage arguments : 1:po_topic_name,  2:float:filter_radius,  3:filter_neighbour,    4:float:filter_voxel_size, 5:float:Ransac_dist_thres";
    //cout<<"  5:cluster_min_size,  6:cluster_max_size,  7:float:normal_search_radius,";
    //cout<<"  8:normal_search_neigh,   9:float:SmoothnessThreshold,   10:float:CurvatureThreshold,";
    //cout<<"  11:reg_Neighbours\n\n";
    
  } else {

    po_topic_name= arguments[1];
    filter_radius= atof (arguments[2]);
    filter_neighbour = atoi (arguments[3]);
    filter_voxel_size= atof (arguments[4]);
    Ransac_dist_thres= atof (arguments[5]);
    //cluster_min_size= atoi (arguments[5]);
    //cluster_max_size= atoi (arguments[6]);
    //normal_search_radius= atof (arguments[7]);
    //normal_search_neigh= atoi (arguments[8]);
    //SmoothnessThreshold= atof (arguments[9]);
    //CurvatureThreshold= atof (arguments[10]);
    //reg_Neighbours= atoi (arguments[11]);
    
    do_region_flow_cluster=false;
    updated=false;
    filter_voxel=false;
    if(filter_voxel_size>0)
      filter_voxel=true;
    
    
    po_sub = ao_nh.subscribe (po_topic_name, 1, &MySubscriber::cloud_cb, this );
    pub = ao_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    // Spin
    //ros::spin ();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_filtered_final_ptr= cloud_filtered_final.makeShared();
    
    //viewer setup
    pcl::visualization::PCLVisualizer viewer ("Point Cloud"); 
    viewer.addCoordinateSystem(10.0);
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);  
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o,0.25,"sphere",0);
    viewer.addPointCloud(cloud_filtered_final_ptr);
    //viewer.setBackgroundColor (1.0, 0.5, 1.0);
    
    while (!viewer.wasStopped ())
    {
      ros::spinOnce();
      if (updated) {
        updated=false;
        cloud_filtered_final_ptr= cloud_filtered_final.makeShared();
        //cout<<"displaying "<<cloud_filtered_final_ptr->width<<" "<<cloud_filtered_final_ptr->height<<"\n";
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_target (cloud_filtered_final_ptr, 0, 255, 0); 
        viewer.updatePointCloud(cloud_filtered_final_ptr, green_target);
        viewer.spinOnce (1);
        //while(1);
      }
    }
    
  }
}

void MySubscriber::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)  
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (Ransac_dist_thres);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    cout<<"Could not estimate a planar model for the given dataset.";
    //return (-1);
  }

  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  std::cout<< "Size of inliners: "<<inliers->indices.size ()<<std::endl;
}
////////////////////////////////////////////////////////////////////////
void MySubscriber::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //intensity data getting lost
  updated=true;
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
  
  //clustering 
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");do
  if(do_region_flow_cluster)
    region_flow_cluster (cloud_filtered);

  // radius filtering
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud (cloud_filtered);
  outrem.setRadiusSearch (filter_radius);
  outrem.setMinNeighborsInRadius (filter_neighbour);
  outrem.filter (*cloud_filtered);
  
  //ransac plane fitting
  fitPlane(cloud_filtered);

  // Publish the data
  sensor_msgs::PointCloud2 output;
  PcltoROSMSG(cloud_filtered,output);
  
  //cloud_filtered_final=cloud;
  copyPointCloud(cloud, cloud_filtered_final);
  //cout<<"size "<<cloud.width<<" "<<cloud.height<<"\n";
  pub.publish (output);
}

////////////////////////////////////////////////////////////////////////

void MySubscriber::region_flow_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  //normal_estimator.setKSearch (normal_search_neigh);
  normal_estimator.setRadiusSearch (normal_search_radius);
  normal_estimator.compute (*normals);

  //pcl::IndicesPtr indices (new std::vector <int>);
  //pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);



  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (cluster_min_size);
  reg.setMaxClusterSize (cluster_max_size);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (reg_Neighbours);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (SmoothnessThreshold / 180.0 * M_PI);
  reg.setCurvatureThreshold (CurvatureThreshold);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  //int counter = 0;
  for(int i=0; i<clusters.size(); i++)
  { 
    std::cout << "cluster has " << clusters[i].indices.size () <<std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  //return (0);  
  sleep(1);
}


////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  MySubscriber s(nh,argc, argv);

  
}
