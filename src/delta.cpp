#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl/point_cloud.h>

#define CENTERED_THRESHOLD 0.3

#define STRAIGHT_THRESHOLD 0.2
#define MIN_TURN_THRESHOLD 0.4
#define TURN_THRESHOLD 1

#define MIN_TURN 0.2
#define TURN 0.3
#define MAX_TURN 0.4

float delta = 0.0;


float notDetected(float v){
  if (isnan(v) || v == 0){
    return 1;
  } else {
    return 0;
  }
}

float oneLineFollow(float w, float b){
  ROS_INFO(" 1 LINE DETECTED ");

  float del = 0;
  
  if(abs(b) < CENTERED_THRESHOLD){
    if (abs(w) < STRAIGHT_THRESHOLD){
      ROS_INFO("NOT CENTERED IN 1 LINE GO STRAIGHT");
      del = 0;
    } else {
      ROS_INFO("NOT CENTERED IN 1 LINE && TURN");
      del = (w > 0 ? TURN : -TURN);

    }
    
  } else {
    if (abs(w) < STRAIGHT_THRESHOLD)
    {
      ROS_INFO("CENTERED IN 1 LINE GO STRAIGHT");
      del = 0;
    } 
    else if (abs(w) < MIN_TURN_THRESHOLD){
      ROS_INFO("CENTERED IN 1 LINE MIN TURN");
      del = MIN_TURN;
    }
    else if (abs(w) < TURN_THRESHOLD){
      ROS_INFO("CENTERED IN 1 LINE TURN");
      del = TURN;
    } else {
      ROS_INFO("CENTERED IN 1 LINE MAX TURN");
      del = MAX_TURN;
    }
    
  }
  return del;
}

float getDelta(float w0, float b0, pcl::PointCloud<pcl::PointXYZ> cloud0, float w1, float b1, pcl::PointCloud<pcl::PointXYZ> cloud1)
{
  // don't know which is left or right
  if (notDetected(b0) && notDetected(b1)){
    ROS_INFO("=========== BOTH LINE NOT DETECTED ==============");
    return delta;
  }
  int left_cloud_size;
  int right_cloud_size;

  float lw0, rw0, lw1, rw1;
  if(!notDetected(b0) && !notDetected(b1)){
    if (b0 > b1)
    {
      lw0 = w0;
      lw1 = b0;
      left_cloud_size = cloud0.size();
      rw0 = w1;
      rw1 = b1;
      right_cloud_size = cloud1.size();

    }
    else
    {
      lw0 = w1;
      lw1 = b1; 
      left_cloud_size = cloud1.size();
      rw0 = w0;
      rw1 = b0;
      right_cloud_size = cloud0.size();
    }

    float mean_slope = (lw0 * left_cloud_size + rw0 * right_cloud_size) / (left_cloud_size + right_cloud_size);

    if (abs(lw1) < CENTERED_THRESHOLD || abs(rw1) < CENTERED_THRESHOLD)
    {
      ROS_INFO("NOT CENTERED....... AND ");
      if (abs(lw1) < CENTERED_THRESHOLD)
      {
        ROS_INFO("LEFT CENTERED! %f", lw1);
        
        if(mean_slope < 0 ){
          ROS_INFO("Left centered RIGHT TURN");
          delta = -MAX_TURN;
        }
        else if (mean_slope > 0 ){
          ROS_INFO("Left centered LEFT TURN");
          delta = MIN_TURN;
        }
      }
      else
      {
        ROS_INFO("RIGHT CENTERED! %f", lw1);
        if(mean_slope < 0 ){
          ROS_INFO("Right centered RIGHT TURN");
          delta = MIN_TURN;
        }
        else if (mean_slope > 0 ){
          ROS_INFO("Right centered LEFT TURN");
          delta = MAX_TURN;
        }
      }
    }
    else
    {
      if (abs(mean_slope) > STRAIGHT_THRESHOLD && abs(mean_slope) < MIN_TURN_THRESHOLD)
      {
      ROS_INFO(" CENTERED  AND  MIN_TURN");
        delta = (mean_slope > 0 ? MIN_TURN : -MIN_TURN);
      }
      else if (abs(mean_slope) >= MIN_TURN_THRESHOLD && abs(mean_slope) < TURN_THRESHOLD)
      {
        ROS_INFO(" CENTERED  AND  TURN");
        delta = (mean_slope > 0 ? TURN : -TURN);
      }
      else if (abs(mean_slope) >= TURN_THRESHOLD)
      {
        ROS_INFO(" CENTERED  AND  MAX_TURN");
        delta = (mean_slope > 0 ? MAX_TURN : -MAX_TURN);
      }
      else {
        ROS_INFO(" CENTERED  AND  STRAIGHT");
        delta = 0;
      }
    }
  } else {
    // 1 line detected
    if (!notDetected(b0)){
      delta = oneLineFollow(w0, b0);
    } else {
      delta = oneLineFollow(w1, b1);
      
    }
  }
  ROS_INFO("============= DELTA %f =====================", delta);

  // velocity control by delta value
  return delta;
}
