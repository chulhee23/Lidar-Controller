#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "laser_geometry/laser_geometry.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ros::Publisher pubLeft;
ros::Publisher pubRight;

laser_geometry::LaserProjection projector;

void callbackFcn(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //Convert sensor_msgs::LaserScan to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(*msg, cloud, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

  pcl::PointCloud<pcl::PointXYZ> inputCloud1;
  pcl::PointCloud<pcl::PointXYZ> voxelCloud;

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(cloud, inputCloud1);

  //Voxelization
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(inputCloud1.makeShared());
  vox.setLeafSize(0.2f, 0.2f, 0.2f); // set Grid Size(0.2m)
  vox.filter(voxelCloud);

  //passthrough
  pcl::PointCloud<pcl::PointXYZ> passCloud1;
  pcl::PointCloud<pcl::PointXYZ> passCloud2;

  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(voxelCloud.makeShared());
  pass.setFilterFieldName("x"); // axis x
  pass.setFilterLimits(-2, 0);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloud1);

  pass.setInputCloud(passCloud1.makeShared());
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-2, 2);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloud2);

  //kdtree

  pcl::PointCloud<pcl::PointXYZ> kdCloudLeft;
  pcl::PointCloud<pcl::PointXYZ> kdCloudRight;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLeft;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeRight;

  kdtreeLeft.setInputCloud(passCloud2.makeShared());
  kdtreeRight.setInputCloud(passCloud2.makeShared());

  pcl::PointXYZ searchPointLeft; // set Search Left Point : (0, -2, 0)
  searchPointLeft.x = 0;
  searchPointLeft.y = -1;
  searchPointLeft.z = 0;

  pcl::PointXYZ searchPointRight; // set Search Right Point : (0, 2, 0)
  searchPointRight.x = 0;
  searchPointRight.y = 1;
  searchPointRight.z = 0;

  std::vector<int> pointIdxRadiusSearch;         //index
  std::vector<float> pointRadiusSquaredDistance; //distance

  float radius = 1.2; // set searching radius size = 2.2m

  if (kdtreeLeft.radiusSearch(searchPointLeft, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
      kdCloudLeft.points.push_back(passCloud2.points[pointIdxRadiusSearch[i]]);

  if (kdtreeRight.radiusSearch(searchPointRight, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
      kdCloudRight.points.push_back(passCloud2.points[pointIdxRadiusSearch[i]]);

  sensor_msgs::PointCloud2 outputLeft;
  sensor_msgs::PointCloud2 outputRight;

  pcl::toROSMsg(kdCloudLeft, outputLeft);
  pcl::toROSMsg(kdCloudRight, outputRight);

  outputLeft.header.frame_id = "/laser";
  outputRight.header.frame_id = "/laser";

  pubLeft.publish(outputLeft);
  pubRight.publish(outputRight);
}

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "filter");
//   ros::NodeHandle nh;

//   // << Subscribe Topic >>
//   // topic name : /scan
//   // topic type : sensor_msgs::LaserScan
//   ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, callbackFcn);

//   // << Publish Topic >>
//   // topic name : /passPC
//   // topic type : sensor_msgs::PointCloud2
//   pubLeft = nh.advertise<sensor_msgs::PointCloud2>("/filterLeft", 1);

//   pubRight = nh.advertise<sensor_msgs::PointCloud2>("/filterRight", 1);

//   ros::spin();
//   return 0;
// }
