#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "gradientDescent.cpp"
#include "delta_tan.cpp"

#define FORWARD_RANGE 2
#define WIDTH 1.5



ros::Publisher point_pub;
ros::Publisher left_pub;
ros::Publisher right_pub;

ros::Publisher line_pub;
ros::Publisher del_pub;
ros::Publisher vel_pub;

void drawLine(LineComponent leftLine, LineComponent rightLine)
{
  visualization_msgs::Marker line;
  line.header.frame_id = "map";
  line.header.stamp = ros::Time::now();
  line.ns = "points_and_lines";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_LIST;
  line.action = visualization_msgs::Marker::ADD;
  line.scale.x = 0.05;
  line.scale.y = 0;
  line.scale.z = 0;
  line.color.a = 1.0;
  line.color.b = 1.0;
  line.pose.orientation.w = 0.5;

  LineComponent lines[2] = {leftLine, rightLine};
  for (int i = 0; i < 2; i++)
  {
    LineComponent tmp_line = lines[i];
    if (notDetected(tmp_line.w0) || notDetected(tmp_line.w1))
    {
      continue;
    }
    else
    {
      geometry_msgs::Point p1;
      p1.x = -1;
      p1.y = tmp_line.w0 * p1.x + tmp_line.w1;
      p1.z = 0;
      line.points.push_back(p1);
      
      geometry_msgs::Point p2;
      p2.x = 5;
      p2.y = tmp_line.w0 * p2.x + tmp_line.w1;
      p2.z = 0;
      line.points.push_back(p2);
      
      line_pub.publish(line);
    }
  }
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud ================
  sensor_msgs::PointCloud msgCloud;
  laser_geometry::LaserProjection projector_;

  projector_.projectLaser(*scan, msgCloud);

  pcl::PointCloud<pcl::PointXYZ> inputCloud;
  inputCloud.width = msgCloud.points.size();
  inputCloud.height = 1;
  inputCloud.points.resize(inputCloud.width * inputCloud.height);
  for (int i = 0; i < msgCloud.points.size(); i++)
  {
    inputCloud.points[i].x = msgCloud.points[i].x;
    inputCloud.points[i].y = msgCloud.points[i].y;
    inputCloud.points[i].z = 0;
  }

  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud end =============

  // filter ROI start ++++++++++++++++++++++++++

  pcl::PointCloud<pcl::PointXYZ> voxelCloud;

  //Voxelization -----------
  pcl::VoxelGrid<pcl::PointXYZ> vox;

  // sensor_msgs
  vox.setInputCloud(inputCloud.makeShared());
  vox.setLeafSize(0.1f, 0.1f, 0.1f); // set Grid Size(0.4m)
  vox.filter(voxelCloud);

  // ========
  pcl::PointCloud<pcl::PointXYZ> tmpCloud;
  pcl::PointCloud<pcl::PointXYZ> passCloud;

  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(voxelCloud.makeShared());
  pass.setFilterFieldName("x"); // axis x
  pass.setFilterLimits(0, FORWARD_RANGE);
  pass.setFilterLimitsNegative(false);
  pass.filter(tmpCloud);

  pass.setInputCloud(tmpCloud.makeShared());
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-WIDTH, WIDTH);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloud);
  ROS_INFO("Pass Cloud Size %i", passCloud.size());

  sensor_msgs::PointCloud2 filteredOutput;
  pcl::toROSMsg(passCloud, filteredOutput);
  filteredOutput.header.frame_id = "/map";
  point_pub.publish(filteredOutput);
  // =============================

  // =======

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(passCloud.makeShared());

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  ec.setClusterTolerance(0.7); // set distance threshold = 1.5m
  // size < min_value -> noise -> not cluster
  ec.setMinClusterSize(4);    // set Minimum Cluster Size
  ec.setMaxClusterSize(1000); // set Maximum Cluster Size

  ec.setSearchMethod(kdtree);
  ec.setInputCloud(passCloud.makeShared());

  ec.extract(clusterIndices);

  // pcl::PointCloud<pcl::PointXYZ> clusteredLeft;
  // pcl::PointCloud<pcl::PointXYZ> clusteredRight;

  pcl::PointCloud<pcl::PointXYZ> clustered[2];

  ROS_INFO("cluster number %i", clusterIndices.size());

  int cluster_idx = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      clustered[cluster_idx].push_back(passCloud[*pit]);
    
    cluster_idx++;
    if (cluster_idx > 1)
    {
      break;
    }
  }

  // clustering end ======================
  ROS_INFO("clustered 1st size %i", clustered[0].size());
  ROS_INFO("clustered 2nd size %i", clustered[1].size());
  if(clustered[0].size() > 0){
   ROS_INFO("clustered first x %f", clustered[0].points[clustered[0].size() - 1].x);
   ROS_INFO("clustered first y %f", clustered[0].points[clustered[0].size() - 1].y);
  }
  LineComponent leftLine = getLine(clustered[0]);
  LineComponent rightLine = getLine(clustered[1]);

  drawLine(leftLine, rightLine);

  ROS_INFO("left w0 %f", leftLine.w0);
  ROS_INFO("left w1 %f", leftLine.w1);
  ROS_INFO("right w0 %f", rightLine.w0);
  ROS_INFO("right w1 %f", rightLine.w1);

  // control delta value

  std_msgs::Float64 delta;
  delta.data = getDelta(leftLine.w0, leftLine.w1, clustered[0], rightLine.w0, rightLine.w1, clustered[1]);

  if (delta.data == 0){
    delta.data -= 0.03;
  }
  
  del_pub.publish(delta);

  sensor_msgs::PointCloud2 outputLeft;
  pcl::toROSMsg(clustered[0], outputLeft);
  outputLeft.header.frame_id = "/map";
  left_pub.publish(outputLeft);

  sensor_msgs::PointCloud2 outputRight;
  pcl::toROSMsg(clustered[1], outputRight);
  outputRight.header.frame_id = "/map";
  right_pub.publish(outputRight);


  // ==========
  std_msgs::Float64 velocity;
  float a = 0.1;
  float v1 = 6;
  float b = 0.35;
  float v2 = 2.7;
 
  if (abs(delta.data) < a) {
    velocity.data = v1;
  } else if (abs(delta.data) < b ){    
    velocity.data = ((v2 - v1)/(b-a)) * (abs(delta.data) - a) + v1;
  } else {
    velocity.data = v2;
  }
  vel_pub.publish(velocity);
  ROS_INFO("VELOCITY %f", velocity.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10); // 0.1 seconds

  // for Arduino publish start ================================
  // The second parameter to advertise() is the size of the message queue used for publishing messages.
  // If messages are published more quickly than we can send them,
  // the number here specifies how many messages to buffer up before throwing some away.
  vel_pub = nh.advertise<std_msgs::Float64>("/vel", 1000);
  del_pub = nh.advertise<std_msgs::Float64>("/del", 1000);

  point_pub = nh.advertise<sensor_msgs::PointCloud2>("/rvizTest", 1);

  left_pub = nh.advertise<sensor_msgs::PointCloud2>("/leftLine", 1);
  right_pub = nh.advertise<sensor_msgs::PointCloud2>("/rightLine", 1);

  line_pub = nh.advertise<visualization_msgs::Marker>("/line", 1);
  // for Arduino publish end ================================

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  std_msgs::Float64 velocity;
  velocity.data = 7;
  vel_pub.publish(velocity);
  ROS_INFO("VELOCITY %f", velocity.data);

  // while(ros::ok()){
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ros::spin();
  return 0;


}
