#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <vector>

#include "gradientDescent.cpp"


#define FORWARD_RANGE 2
#define WIDTH 1

ros::Publisher point_pub;
ros::Publisher line_pub;
ros::Publisher del_pub;


float get_delta(float lw0, float lw1, float rw0, float rw1){
  

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
  pcl::PointCloud<pcl::PointXYZ> tmpCloud;
  pcl::PointCloud<pcl::PointXYZ> passCloudLeft;
  pcl::PointCloud<pcl::PointXYZ> passCloudRight;


  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(voxelCloud.makeShared());
  pass.setFilterFieldName("x"); // axis y
  pass.setFilterLimits(-1.5, 1.5);
  pass.setFilterLimitsNegative(false);
  pass.filter(tmpCloud);

  pass.setInputCloud(tmpCloud.makeShared());
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(0, 1.5);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloudLeft);

  pass.setInputCloud(tmpCloud.makeShared());
  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-1.5, 0);
  pass.setFilterLimitsNegative(false);
  pass.filter(passCloudRight);

  // clustering start=====================
  
  // left+++++++++++++++++++++++++++++++++++++
  pcl::PointCloud<pcl::PointXYZ> filteredLeft;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(passCloudLeft.makeShared());
  std::vector<pcl::PointIndices> clusterIndices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_left;
  ec_left.setClusterTolerance(0.6); // set distance threshold = 0.9m

  ec_left.setMinClusterSize(2);   // set Minimum Cluster Size
  ec_left.setMaxClusterSize(100); // set Maximum Cluster Size
  ec_left.setSearchMethod(kdtree);
  ec_left.setInputCloud(passCloudLeft.makeShared());
  ec_left.extract(clusterIndices);

  std::vector<pcl::PointIndices>::const_iterator it;

  for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
  {

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      filteredLeft.push_back(passCloudLeft[*pit]);
  }
  // left end +++++++++++++++++++++++++++++++++++++


  // right start -----------------------
  pcl::PointCloud<pcl::PointXYZ> filteredRight;

  kdtree->setInputCloud(passCloudRight.makeShared());

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_right;
  ec_right.setClusterTolerance(0.6); // set distance threshold = 0.9m

  ec_right.setMinClusterSize(2);   // set Minimum Cluster Size
  ec_right.setMaxClusterSize(100); // set Maximum Cluster Size
  ec_right.setSearchMethod(kdtree);
  ec_right.setInputCloud(passCloudRight.makeShared());
  ec_right.extract(clusterIndices);

  for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
  {
    
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      filteredRight.push_back(passCloudRight[*pit]);
  }

  // right end -----------------------


  // clustering end ======================
  LineComponent leftLine = getLine(filteredLeft);
  LineComponent rightLine = getLine(filteredRight);

  ROS_INFO("left w0 %f", leftLine.w0);
  ROS_INFO("left w1 %f", leftLine.w1);
  ROS_INFO("right w0 %f", rightLine.w0);
  ROS_INFO("right w1 %f", rightLine.w1);

  // control delta value

  std_msgs::Float64 delta;
  delta.data = get_delta(leftLine.w0, leftLine.w1, rightLine.w0, rightLine.w1);
  del_pub.publish(delta);

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


  LineComponent lines[2]  = {leftLine, rightLine};
  for(int i = 0 ; i < 2; i++){
    LineComponent tmp_line = lines[i];
    if (isnan(tmp_line.w0) || isnan(tmp_line.w1))
    {
      ROS_INFO("======= slope 0 !!!!!!!! %f ===========", tmp_line.w0);
    } else {
      geometry_msgs::Point p1;
      p1.x = -5; p1.y = tmp_line.w0 * (-5) + tmp_line.w1; p1.z = 0;

      line.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = 1; p2.y = tmp_line.w0 + tmp_line.w1; p2.z = 0;

      line.points.push_back(p2);
      line_pub.publish(line);
    }
  }


  //
  sensor_msgs::PointCloud2 outputLeft;
  sensor_msgs::PointCloud2 outputRight;
  
  pcl::toROSMsg(filteredLeft, outputLeft);
  output.header.frame_id = "/map";
  pub.publish(outputLeft);
  
  pcl::toROSMsg(filteredLeft, outputRight);
  output.header.frame_id = "/map";
  pub.publish(outputRight);
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
