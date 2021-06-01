#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl/point_cloud.h>

#define CENTERED_THRESHOLD 0.4
#define MICRO_TURN 0.1
#define SLOPE_WEIGHT 1.15
// max del : 0.5233

float delta = 0.0;

float notDetected(float v)
{
  if (isnan(v) || v == 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


float purePursuit(float w){
  float car_axis_distance = 0.257;
  float alpha = atan2f(w, 1);
  float Ld = 0.52;
  // Ld lower -> 횡방향
  // Ld higher -> 조향 감소
  return atan2f(2 * car_axis_distance * sin(alpha), Ld) * 1.2;

}


float oneLineFollow(float w, float b)
{
  ROS_INFO(" 1 LINE DETECTED ");
  float del = 0;

  // pure pursuit algorithm
  del = purePursuit(w);

  if (abs(b) < CENTERED_THRESHOLD){
    if (abs(delta) < 0.3){
      if(b < 0){
        del += MICRO_TURN;
      } else {
        del -= MICRO_TURN;
      }
    }
  }

  return del;
}

float getDelta(float w0, float b0, pcl::PointCloud<pcl::PointXYZ> cloud0, float w1, float b1, pcl::PointCloud<pcl::PointXYZ> cloud1)
{
  // don't know which is left or right
  if (notDetected(w0) && notDetected(w1))
  {
    ROS_INFO("=========== BOTH LINE NOT DETECTED ==============");
    return delta;
  }
  int left_cloud_size;
  int right_cloud_size;

  float lw0, rw0, lw1, rw1;
  if (!notDetected(w0) && !notDetected(w1))
  {
    // both line detected ==========================================
    // setting left and right
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

    // default delta
    // pure pursuit algorithm
    // delta = atan2f(mean_slope * SLOPE_WEIGHT, 1);
    delta = purePursuit(mean_slope);
    

    // y 절편

    if (abs(delta) < 0.25){
      if(abs(rw1) < CENTERED_THRESHOLD){
        delta += MICRO_TURN;
      }
      if(abs(lw1) < CENTERED_THRESHOLD){
        delta -= MICRO_TURN;
      }
    }    

  }
  else
  {
    // 1 line detected
    if (!notDetected(w0))
    {
      delta = oneLineFollow(w0, b0);
    }
    else
    {
      delta = oneLineFollow(w1, b1);
    }
  }

  
  ROS_INFO("============= DELTA %f =====================", delta);

  // velocity control by delta value
  return delta;
}
