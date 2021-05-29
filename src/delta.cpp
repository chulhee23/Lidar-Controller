#include <ros/ros.h>
#include <std_msgs/Float64.h>


#define CENTERED_THRESHOLD 0.3

#define STRAIGHT_THRESHOLD 0.2
#define MIN_TURN_THRESHOLD 0.4
#define TURN_THRESHOLD 1

#define MIN_TURN 0.15
#define TURN 0.3
#define MAX_TURN 0.5

float delta = 0.0;


float notDetected(float v){
  if (isnan(v) || v == 0){
    return 1;
  } else {
    return 0;
  }
}

float oneLineFollow(float w, float b){
  float del = w;
  if(abs(b) < CENTERED_THRESHOLD){
    ROS_INFO("MIN TURN in ONE LINE");
    del = (w > 0 ? MIN_TURN : -MIN_TURN);
  } else {
    ROS_INFO("TURN in ONE LINE");
    del = (w > 0 ? TURN : -TURN);
  }
  return del;
}

float getDelta(float w0, float b0, float w1, float b1){
  // don't know which is left or right
  if (notDetected(b0) && notDetected(b1)){
    ROS_INFO("=========== BOTH LINE NOT DETECTED ==============");
    return delta;
  }

  float lw0, rw0, lw1, rw1;
  if(!notDetected(b0) && !notDetected(b1)){
    if (b0 > b1)
    {
      lw0 = w0;
      lw1 = b0;
      rw0 = w1;
      rw1 = b1;
    }
    else
    {
      lw0 = w1;
      lw1 = b1; 
      rw0 = w0;
      rw1 = b0;
    }

    float mean_slope = (lw0 + rw0) / 2;

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
          delta = -MIN_TURN;
        }
        else if (mean_slope > 0 ){
          ROS_INFO("Right centered LEFT TURN");
          delta = MAX_TURN;
        }
      }
    }
    else
    {
      ROS_INFO("==== CENTERED ");
      
      if (abs(mean_slope) > STRAIGHT_THRESHOLD && abs(mean_slope) < MIN_TURN_THRESHOLD)
      {
        delta = (mean_slope > 0 ? MIN_TURN : -MIN_TURN);
      }
      else if (abs(mean_slope) >= MIN_TURN_THRESHOLD && abs(mean_slope) < TURN_THRESHOLD)
      {
        delta = (mean_slope > 0 ? TURN : -TURN);
      }
      else if (abs(mean_slope) >= TURN_THRESHOLD)
      {
        delta = (mean_slope > 0 ? MAX_TURN : -MAX_TURN);
      }
      else {
        delta = 0;
      }
    }
  } else {
    // 1 line detected
    ROS_INFO("=========== 1 LINE DETECTED ==============");
    if (!notDetected(b0)){
      // 
      delta = oneLineFollow(w0, b0);
    } else {
      delta = oneLineFollow(w1, b1);
      
    }
  }
  ROS_INFO("====== DELTA %f =========", delta);

  // velocity control by delta value
  return delta;  

}
