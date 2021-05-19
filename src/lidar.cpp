#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "gradientDescent.cpp"


const int std_msgs::Float64 velocity = 3;
ros::Publisher del_pub;

float 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud ================
  sensor_msgs::PointCloud msgCloud;
  laser_geometry::LaserProjection projector_;
  projector_.projectLaser(*scan, msgCloud);

  // voxel

  // clustering

  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud end =============


  // float slope = getGradientDescent(msgCloud.points.x, msgCloud.points.y);
  
  std_msgs::Float64 delta;
  // delta = getDelta(slope);
  // del_pub.publish(delta);

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

  vel_pub.publish(velocity);

  // for Arduino publish end ================================

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  // while Lidar On
  // 1. get slope by gradient descent
  

  // 2. publish delta value to Arduino


  ros::spin();
  
  // while(ros::ok()){
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}