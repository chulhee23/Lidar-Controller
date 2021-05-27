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


#define FORWARD_RANGE 3
#define WIDTH 1.5
#define WIDTH_Y_START -0.25
#define DISTANCE_THRESHOLD 0.35


ros::Publisher point_pub;
ros::Publisher left_pub;
ros::Publisher right_pub;

ros::Publisher line_pub;
ros::Publisher del_pub;

void drawLine(LineComponent leftLine, LineComponent rightLine){
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
    if (isnan(tmp_line.w0) || isnan(tmp_line.w1))
    {
      ROS_INFO("======= slope 0 !!!!!!!! %f ===========", tmp_line.w0);
    }
    else
    {
      geometry_msgs::Point p1;
      p1.x = -5;
      p1.y = tmp_line.w0 * (-5) + tmp_line.w1;
      p1.z = 0;

      line.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = 1;
      p2.y = tmp_line.w0 + tmp_line.w1;
      p2.z = 0;

      line.points.push_back(p2);
      line_pub.publish(line);
    }
  }
}


float get_delta(float lw0, float lw1, float rw0, float rw1){
  float del = 0;
  
  if(((lw0 + rw0)/2) > 0.28){
    del = 0.3;
  } 
  else if (((lw0 + rw0)/2) < -0.28) {
    del = -0.3;
  }
  
  return del;
}


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
  pass.setFilterLimits(-FORWARD_RANGE, 0);
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
  pcl::PointCloud<pcl::PointXYZ> kdCloudLeft;
  pcl::PointCloud<pcl::PointXYZ> kdCloudRight;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLeft;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeRight;

  kdtreeLeft.setInputCloud(passCloud.makeShared());
  kdtreeRight.setInputCloud(passCloud.makeShared());

  pcl::PointXYZ searchPointLeft; // set Search Left Point : (0, -2, 0)
  searchPointLeft.x = -1;
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
      kdCloudLeft.points.push_back(passCloud.points[pointIdxRadiusSearch[i]]);

  if (kdtreeRight.radiusSearch(searchPointRight, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
      kdCloudRight.points.push_back(passCloud.points[pointIdxRadiusSearch[i]]);
  ROS_INFO("Kd Left Cloud Size %i", kdCloudLeft.size());
  ROS_INFO("Kd Right Cloud Size %i", kdCloudRight.size());
  // =======

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(kdCloudLeft.makeShared());

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  ec.setClusterTolerance(0.35); // set distance threshold = 1.5m
  // size < min_value -> noise -> not cluster
  ec.setMinClusterSize(2);    // set Minimum Cluster Size
  ec.setMaxClusterSize(1000); // set Maximum Cluster Size

  ec.setSearchMethod(kdtree);
  ec.setInputCloud(kdCloudLeft.makeShared());

  ec.extract(clusterIndices);
  std::vector<pcl::PointIndices>::const_iterator it;
  pcl::PointCloud<pcl::PointXYZ> clusteredLeft;

  ROS_INFO("left clusterIndices %i", clusterIndices.size());
  int clusterN = 1;
  for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
  {  
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      clusteredLeft.push_back(kdCloudLeft[*pit]);

    if (clusterN > 1)
      break;
    
    clusterN++;
  }

  // clustering end ======================
  LineComponent leftLine = getLine(clusteredLeft);
  LineComponent rightLine = getLine(kdCloudRight);



  drawLine(leftLine, rightLine);

  ROS_INFO("left w0 %f", leftLine.w0);
  ROS_INFO("left w1 %f", leftLine.w1);
  ROS_INFO("right w0 %f", rightLine.w0);
  ROS_INFO("right w1 %f", rightLine.w1);
  
  // control delta value

  std_msgs::Float64 delta;
  delta.data = get_delta(leftLine.w0, leftLine.w1, rightLine.w0, rightLine.w1);
  ROS_INFO("====== delta %f =========", delta);
  del_pub.publish(delta);
  

  sensor_msgs::PointCloud2 outputLeft;
  pcl::toROSMsg(clusteredLeft, outputLeft);
  outputLeft.header.frame_id = "/map";
  left_pub.publish(outputLeft);
  
  sensor_msgs::PointCloud2 outputRight;
  pcl::toROSMsg(kdCloudRight, outputRight);
  outputRight.header.frame_id = "/map";
  right_pub.publish(outputRight);

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

  left_pub = nh.advertise<sensor_msgs::PointCloud2>("/leftLine", 1);
  right_pub = nh.advertise<sensor_msgs::PointCloud2>("/rightLine", 1);


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
