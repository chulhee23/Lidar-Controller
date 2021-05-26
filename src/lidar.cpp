#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <vector>

#include "gradientDescent.cpp"


#define FORWARD_RANGE 2
#define WIDTH 1

ros::Publisher point_pub;
ros::Publisher line_pub;
ros::Publisher del_pub;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
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
  // pcl::fromROSMsg(msgCloud, inputCloud);
  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud end =============

  // filter ROI start ++++++++++++++++++++++++++
  
  pcl::PointCloud<pcl::PointXYZ> voxelCloud;

  //Voxelization -----------
  pcl::VoxelGrid<pcl::PointXYZ> vox;

  // sensor_msgs 
  vox.setInputCloud(inputCloud.makeShared());
  vox.setLeafSize(0.4f, 0.4f, 0.4f); // set Grid Size(0.4m)
  vox.filter(voxelCloud);

  //passthrough===============
  pcl::PointCloud<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ> filteredCloud;
  
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(voxelCloud.makeShared());
  
  pass.setFilterFieldName("x"); // axis x
  pass.setFilterLimits(-FORWARD_RANGE, 0);
  pass.setFilterLimitsNegative(false);

  
  pass.filter(filter); // pass 로 filtering

  pass.setInputCloud(filter.makeShared());
  
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-WIDTH, WIDTH);  
  pass.setFilterLimitsNegative(false);
  
  pass.filter(filteredCloud); // pass 로 filtering

  

  // clustering start====



  pcl::PointCloud<pcl::PointXYZ> kdCloud;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // clustering end ======================


  // draw line ==========================
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
  line.color.r = 1.0;

  line.pose.orientation.w = 1.0;

  // TODO : change cloud 
  LineComponent lineComponent = getLine(filteredCloud);
  

  if (lineComponent.w0 < 0.001){
    line.color.b = 1.0;
    ROS_INFO("=======slope 0 !!!!!!!! %f ===========", lineComponent.w0)
  } else {
    geometry_msgs::Point p1;
    p1.x = -5;
    p1.y = w0 * (-5) + w1;
    p1.z = 0;

    line.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = 5;
    p2.y = w0 * (5) + w1;
    p2.z = 0;

    line.points.push_back(p2);
  }


  line_pub.publish(line);
  // draw line end ==========================


  // rviz test

  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(filteredCloud, output);
  output.header.frame_id = "/map";
  point_pub.publish(output);


  // getDelta(msgCloud);


  // float slope = getGradientDescent(msgCloud.points.x, msgCloud.points.y);
  
  std_msgs::Float64 delta;
  delta.data = 0.25;
  // delta = getDelta(slope);
  
  // data > 0 -> turn left
  // data < 0 -> turn right
  
  

  ROS_INFO("DELTA %f", delta.data);

  del_pub.publish(delta);

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
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("/vel", 1000);
  del_pub = nh.advertise<std_msgs::Float64>("/del", 1000);
  
  point_pub = nh.advertise<sensor_msgs::PointCloud2>("/rvizTest", 1);

  line_pub = nh.advertise<visualization_msgs::Marker>("/line", 1);
  // for Arduino publish end ================================

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
  
  std_msgs::Float64 velocity;
  velocity.data = 4; 
  vel_pub.publish(velocity);
  ROS_INFO("VELOCITY %f", velocity.data);
  
  // while(ros::ok()){
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ros::spin();
  return 0;
}
