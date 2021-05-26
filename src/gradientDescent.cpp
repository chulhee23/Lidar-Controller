// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <cmath>
// #include <vector>
// #include <numeric>
// #include <algorithm>
// #include <utility>
// using namespace std;

// ros::Publisher pub;

// // float a1, a2, b1, b2;
// // float getLineY(float x1, float x2, float y1, float y2, float x3){
// //   return (float(y2 - y1)/float(x2 - x1) * (x3 - x1) + y1);
// // }

// // const int cloud_size = 16;

// struct LineComponent {
//   float w0;
//   float w1;
// };


// struct Diff
// {
//   float x;
//   float y;
// };

// Diff dmse_line(vector<pcl::PointXYZ> points, float w[], int cloud_size)
// {
//   vector<float> y;
//   Diff diff;

//   // y 구하기
//   // y = w[0] * x + w[1]
//   float c = 0;
//   for (int i = 0; i < cloud_size; i++)
//   {
//     c = w[0] * points[i].x + w[1];
//     y.push_back(c);
//   }

//   // d_w0 = 2 * np.mean((y-t) * x)
//   // d_w1 = 2 * np.mean(y-t)
//   vector<float> d_w0_vec;
//   vector<float> d_w1_vec;

//   for (int i = 0; i < y.size(); i++)
//   {
//     float c = (y[i] - points[i].y) * points[i].x;
//     float d = (y[i] - points[i].y);
//     d_w0_vec.push_back(c);
//     d_w1_vec.push_back(d);
//   }

//   float mean_0 = accumulate(d_w0_vec.begin(), d_w0_vec.end(), 0.0) / d_w0_vec.size();
//   float mean_1 = accumulate(d_w1_vec.begin(), d_w1_vec.end(), 0.0) / d_w1_vec.size();

//   float d_w0 = 2 * mean_0;
//   float d_w1 = 2 * mean_1;

//   diff.x = d_w0;
//   diff.y = d_w1;

//   return diff;
// }



// void msgCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {

//   pcl::PointCloud<pcl::PointXYZ> inputCloud;
//   pcl::fromROSMsg(*msg, inputCloud);

//   // draw linels
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

//   // float *x = new float[cloud_size];
//   // float *t = new float[cloud_size];
//   float w_init[2] = {1, 1};

//   // for(int i = 0; i<cloud_size; i++){
//   //   geometry_msgs::Point p;
//   //   float x[i] = inputCloud.points[i].x;
//   //   float t[i] = inputCloud.points[i].y;
//   // }

//   float alpha = 0.001;
//   float i_max = 100000;
//   float eps = 0.1;
//   int index = 0;

//   //2차원 벡터 생성
//   // vector<pair<int, int>> w_i(i_max, make_pair(0, 0));
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

  
//   // ====================


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
// }
