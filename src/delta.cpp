#include <ros/ros.h>
#include <std_msgs/Float64.h>


#define MIN_TURN 0.3
#define TURN 0.4
#define MAX_TURN 0.5
#define Y_AXIS_THRESHOLD 0.65

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
  if(abs(b) < 0.3){
    del = (w > 0 ? TURN : -TURN);
  } else {
    del = (w > 0 ? MAX_TURN : -MAX_TURN);
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
    
    if (abs(lw1) < 0.3 || abs(rw1) < 0.3){
      ROS_INFO("NOT CENTERED....... AND ");
      if(abs(lw1) < 0.3){
        ROS_INFO("LEFT CENTERED! %f", lw1);
        
        if(mean_slope < 0 ){
          ROS_INFO("Left centered RIGHT TURN");
          delta = -MAX_TURN;
        }
        else if (mean_slope > 0 ){
          ROS_INFO("Left centered LEFT TURN");
          delta = TURN;
        }
      } else {
        ROS_INFO("RIGHT CENTERED! %f", lw1);
        if(mean_slope < 0 ){
          ROS_INFO("Right centered RIGHT TURN");
          delta = -TURN;
        }
        else if (mean_slope > 0 ){
          ROS_INFO("Right centered LEFT TURN");
          delta = MAX_TURN;
        }

      }


    } else {
      ROS_INFO("==== CENTERED ");

      if (abs(mean_slope) < 0.3){
        delta = 0;
      } else if (abs(mean_slope) >= 0.3 && abs(mean_slope) < 1){
        delta = (mean_slope > 0 ? TURN : -TURN);
      } else if (abs(mean_slope) >= 1){
        delta = (mean_slope > 0 ? MAX_TURN : -MAX_TURN);
      }
    }

  } else {
    // 1 line detected
    ROS_INFO("=========== 1 LINE DETECTED ==============");
    if (!notDetected(b0)){
      // 
      delta = oneLineFollow(w0, b0);
    } else {
      ROS_INFO("========");
      delta = oneLineFollow(w1, b1);
      
    }
  }
  ROS_INFO("====== DELTA %f =========", delta);

  // velocity control by delta value
  return delta;  

}

// float get_delta(float w0, float b0, float w1, float b1)
// {
//   // don't know which is left or right
//   float lw0, rw0, lw1, rw1;
//   if (b0 > b1)
//   {
//     lw0 = w0;
//     lw1 = b0;
//     rw0 = w1;
//     rw1 = b1;
//   }
//   else
//   {
//     lw0 = w1;
//     lw1 = b1; // left
//     rw0 = w0;
//     rw1 = b0; // right
//   }

//   // keep center first
//   if (notDetected(lw1) && notDetected(rw1))
//   {
//     ROS_INFO("==== ERROR!!! : BOTH LINE NOT DETECTED ========");
//     // return prev delta
//     delta = 0;
//     return delta;
//   }
//   else if (notDetected(rw1))
//   {
//     ROS_INFO("==== WARNING : RIGHT LINE NOT DETECTED ========");
//     if (abs(lw1) > Y_AXIS_THRESHOLD)
//     {
//       // 오른쪽으로 치우침
//       if (lw0 > 0)
//       {
//         ROS_INFO("Right centered during LEFT TURN");
//         delta = MAX_TURN; // 좌회전 중
//       }
//       else
//       {
//         ROS_INFO("Right centered during RIGHT TURN");
//         delta = MIN_TURN; // 우회전 중
//       }
//     } 
//     else 
//     {
//       if(lw0 > 0){
//         ROS_INFO("LEFT TURN");
//         delta = TURN;
//       } else {
//         ROS_INFO("RIGHT TURN");
//         delta = -TURN;
//       }
//     }
//     return delta;
//   }
//   else if (notDetected(lw1))
//   {
//     ROS_INFO("==== WARNING : LEFT LINE NOT DETECTED ========");
//     if (abs(rw1) > Y_AXIS_THRESHOLD)
//     {
//       // 왼쪽으로 치우침
//       if (rw0 < 0)
//       {
//         ROS_INFO("Left centered during RIGHT TURN");
//         delta = -MAX_TURN; // 좌회전 중
//       }
//       else
//       {
//         ROS_INFO("Left centered during RIGHT TURN");
//         delta = -MIN_TURN; // 우회전 중
//       }
//     }
//     else
//     {
//       if (rw0 > 0)
//       {
//         ROS_INFO("LEFT TURN");
//         delta = TURN;
//       }
//       else
//       {
//         ROS_INFO("RIGHT TURN");
//         delta = -TURN;
//       }
//     }
//     return delta;
//   }

//   // both detected
//   if (abs(lw1) > Y_AXIS_THRESHOLD)
//   {
//     // 오른쪽으로 치우침
//     if (lw0 > 0)
//     {
//       ROS_INFO("Right centered during LEFT TURN");
//       delta = MAX_TURN; // 좌회전 중
//     }
//     else
//     {
//       ROS_INFO("Right centered during RIGHT TURN");
//       delta = MIN_TURN; // 우회전 중
//     }
//   }
//   else if (abs(rw1) > Y_AXIS_THRESHOLD)
//   {
//     // 왼쪽으로 치우침
//     if (lw0 > 0)
//     {
//       ROS_INFO("Case 3: Left centered during LEFT TURN");
//       delta = MIN_TURN; // 좌회전 중
//     }
//     else
//     {
//       ROS_INFO("Case 4: Left centered during RIGHT TURN");
//       delta = MAX_TURN; // 우회전 중
//     }
//   }
//   else
//   {
//     // 적절한 중앙차선 유지 시,
//     if (((lw0 + rw0) / 2) > 0.3)
//     {
//       ROS_INFO("Case 5: LEFT TURN");
//       delta = TURN;
//     }
//     else if (((lw0 + rw0) / 2) < -0.3)
//     {
//       ROS_INFO("Case 6: RIGHT TURN");
//       delta = -TURN;
//     }
//     else
//     {
//       ROS_INFO("Case STRAIGHT");
//       delta = 0;
//     }
//   }
//   return delta;
// }
