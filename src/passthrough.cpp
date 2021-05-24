#include <iostream>
#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;



void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloud<pcl::PointXYZ> ptr_filtered;
  pcl::PassThrough<pcl::PointXYZI> ptfilter;

  pcl::fromROSMsg(*msg,*ptr_filtered);

  // axis x
  //ptfilter.setInputCloud(ptr_filtered);
  //ptfilter.setFilterFieldName("x");
  //ptfilter.setFilterLimits(-1.5,-1.5);
  //ptfilter.setFilterLimitsNegative(true);
  //ptfilter.filter(*ptr_filtered);

  // axis y
  ptfilter.setInputCloud(ptr_filtered);
  ptfilter.setFilterFieldName("y");
  ptfilter.setFilterLimits(-1.5,-1.5);
  ptfilter.setFilterLimitsNegative(true);
  ptfilter.filter(*ptr_filtered);


  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new         pcl::PointCloud<pcl::PointXYZI> ());
  //pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(*transformed_cloud, cloud_p); 


 
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p,output);
 // pcl::toROSMsg(ptfilter, output);
  output.header.frame_id = "/map";

  pub.publish(output);
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "passthrough");
ros::NodeHandle nh;

//ros::Publisher pass_pub = nh.advertise<sensor_msgs::PointCloud2>("/passfilter", 1);

//ros::Subscriber pass_sub = nh.subscribe<senso_msgs::PointCloud2>("/input", 10, callbackFcn);

pub = nh.advertise<sensor_msgs::PointCloud2>("/passfilter",1);



ros::spin();
return 0;
}
  
