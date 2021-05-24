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

ros::Publisher pub;

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
  vox.setLeafSize(0.4f, 0.4f, 0.4f); // set Grid Size(0.4m)
  vox.filter(voxelCloud);

  //passthrough
  pcl::PointCloud<pcl::PointXYZ> passCloud1;
  pcl::PointCloud<pcl::PointXYZ> passCloud2;

  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(voxelCloud.makeShared());
  pass.setFilterFieldName("x"); // axis y
  pass.setFilterLimits(-1.5, 1.5);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloud1);

  pass.setInputCloud(passCloud1.makeShared());
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-1.5, 1.5);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloud2);

  //kdtree

  pcl::PointCloud<pcl::PointXYZ> kdCloud;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(passCloud2.makeShared());

  pcl::PointXYZ searchPoint; // set Search Point : (0, 0, 0)
  searchPoint.x = 0;
  searchPoint.y = 0;
  searchPoint.z = 0;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 0.2; // set searching radius size = 0.2m
  if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
      kdCloud.points.push_back(passCloud2.points[pointIdxRadiusSearch[i]]);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(passCloud2, output);
  output.header.frame_id = "/laser";

  pub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh;

  // << Subscribe Topic >>
  // topic name : /scan
  // topic type : sensor_msgs::LaserScan
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, callbackFcn);

  // << Publish Topic >>
  // topic name : /passPC
  // topic type : sensor_msgs::PointCloud2
  pub = nh.advertise<sensor_msgs::PointCloud2>("/filterPC", 1);

  ros::spin();
  return 0;
}
