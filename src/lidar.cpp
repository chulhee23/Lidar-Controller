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


#define FORWARD_RANGE 5
#define WIDTH 1.5
ros::Publisher pub;
ros::Publisher del_pub;


// std_msgs::Float64 getDelta(float slope){
  // TODO
  
  // return 0;
// }

// void getDelta(const sensor_msgs::PointCloud::ConstPtr msgCloud){
//   visualization_msgs::Marker line;
//   line.header.frame_id = "map";
//   line.header.stamp = ros::Time::now();
//   line.ns = "points_and_lines";
//   line.id = 0;
//   line.type = visualization_msgs::Marker::LINE_LIST;
//   line.action = visualization_msgs::Marker::ADD;
//   line.scale.x = 0.1;
//   line.scale.y = 0;
//   line.scale.z = 0;
//   line.color.a = 1.0;
//   line.color.r = 1.0;
//   line.pose.orientation.w = 1.0;

//   int cloud_size = inputCloud.points.size();
//   float w_init[2] = {1, 1};

//   float alpha = 0.001;
//   float i_max = 100000;
//   float eps = 0.1;
//   int index = 0;

//   vector<pair<float, float>> w_i = {{0, 0}, {0, 0}, {0, 0}};
//   w_i[0] = pair<float, float>(w_init[0], w_init[1]);
//   for (int i = 1; i < i_max; i++)
//   {
//     index++;
//     w_i.push_back(pair<float, float>(0, 0));
//     float w_x = w_i[i - 1].first;
//     float w_y = w_i[i - 1].second;
//     float tmp[2] = {w_x, w_y};
//     Diff dmse = dmse_line(inputCloud.points, tmp, cloud_size);

//     // w_i[i] = (~~, ~~~)
//     // 함수 생성해서 dmse 가져오기 --> 우선 dd_w0/dd_w1 사용
//     float ca = w_i[i - 1].first - alpha * dmse.x;
//     float da = w_i[i - 1].second - alpha * dmse.y;
//     w_i[i] = pair<float, float>(ca, da);
//     // cout << w_i[i].first << " // " << w_i[i].second << endl;
//     // if max(np.absolute(dmse)) < eps: // 종료판정
//     float max_dmse = max(abs(dmse.x), abs(dmse.y));
//     // cout << max_dmse << endl;
//     if (max_dmse < eps)
//     {
//       // cout << dmse.x << " " << dmse.y << endl;
//       break;
//     }
//   }

//   float w0 = w_i[index].first;
//   float w1 = w_i[index].second;

//   geometry_msgs::Point p1;
//   p1.x = -5;
//   p1.y = w0 * (-5) + w1;
//   p1.z = 0;
//   line.points.push_back(p1);

//   geometry_msgs::Point p2;
//   p2.x = 5;
//   p2.y = w0 * 5 + w1;
//   p2.z = 0;
//   line.points.push_back(p2);

//   pub.publish(line);
// };

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
  pcl::PointCloud<pcl::PointXYZ> passCloud;
  
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(voxelCloud.makeShared());
  
  pass.setFilterFieldName("x"); // axis x
  pass.setFilterLimits(-FORWARD_RANGE, 0);
  pass.setFilterLimitsNegative(false);

  pass.setFilterFieldName("y"); // axis y
  pass.setFilterLimits(-WIDTH, WIDTH);  
  pass.setFilterLimitsNegative(false);
  
  
  pass.filter(passCloud); // pass 로 filtering

  // pass.setInputCloud(passCloud1.makeShared());
  // pass.filter(passCloud2);

  // passthrough end =====================
  // filter ROI end ++++++++++++++++++++++++++

  // clustering start ======================
  // TODO

  pcl::PointCloud<pcl::PointXYZ> kdCloud;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // clustering end ======================

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
  

  // for Arduino publish end ================================

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  // while Lidar On
  // 1. get slope by gradient descent
  

  // 2. publish delta value to Arduino


  // ros::spin();
  
  std_msgs::Float64 velocity;
  velocity.data = 4; 
  vel_pub.publish(velocity);
  ROS_INFO("VELOCITY %f", velocity.data);
  
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
