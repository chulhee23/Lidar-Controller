#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <utility>
using namespace std;

// float getLineY(float x1, float x2, float y1, float y2, float x3){
//   return (float(y2 - y1)/float(x2 - x1) * (x3 - x1) + y1);
// }

struct LineComponent {
  float w0;
  float w1;
};

struct Diff
{
  float x;
  float y;
};

Diff dmse_line(pcl::PointCloud<pcl::PointXYZ> points, float w[], int cloud_size)
{
  Diff diff;
  if (cloud_size > 0){
    vector<float> y;

    // y 구하기
    // y = w[0] * x + w[1]
    float c = 0;
    for (int i = 0; i < cloud_size; i++)
    {
      c = w[0] * points[i].x + w[1];
      y.push_back(c);
    }

    // d_w0 = 2 * np.mean((y-t) * x)
    // d_w1 = 2 * np.mean(y-t)
    vector<float> d_w0_vec;
    vector<float> d_w1_vec;

    for (int i = 0; i < y.size(); i++)
    {
      float c = (y[i] - points[i].y) * points[i].x;
      float d = (y[i] - points[i].y);
      d_w0_vec.push_back(c);
      d_w1_vec.push_back(d);
    }

    float mean_0 = 0;
    float mean_1 = 0;
    if (d_w0_vec.size() > 0){
      mean_0 = accumulate(d_w0_vec.begin(), d_w0_vec.end(), 0.0) / d_w0_vec.size();

    }
    if (d_w1_vec.size() > 0){
      mean_1 = accumulate(d_w1_vec.begin(), d_w1_vec.end(), 0.0) / d_w1_vec.size();
    }

    float d_w0 = 2 * mean_0;
    float d_w1 = 2 * mean_1;

    diff.x = d_w0;
    diff.y = d_w1;


  } else {
    diff.x = 0;
    diff.y = 0;

  }
  return diff;

}

    

LineComponent getLine(const pcl::PointCloud<pcl::PointXYZ> inputCloud){
  int cloud_size = inputCloud.points.size();
  float w_init[2] = {0, 0};

  float alpha = 0.01;
  float i_max = 1000;
  float eps = 0.001;
  int index = 0;
  vector<pair<float, float>> w_i = {{0, 0}, {0, 0}, {0, 0}};

  w_i[0] = pair<float, float>(w_init[0], w_init[1]);

  for (int i = 1; i < i_max; i++)
  {
    index++;
    w_i.push_back(pair<float, float>(0, 0));
    float w_x = w_i[i - 1].first;
    float w_y = w_i[i - 1].second;
    float tmp[2] = {w_x, w_y};
    Diff dmse = dmse_line(inputCloud, tmp, cloud_size);

    // w_i[i] = (~~, ~~~)
    // 함수 생성해서 dmse 가져오기 --> 우선 dd_w0/dd_w1 사용
    float ca = w_i[i - 1].first - alpha * dmse.x;
    float da = w_i[i - 1].second - alpha * dmse.y;
    w_i[i] = pair<float, float>(ca, da);
    // cout << w_i[i].first << " // " << w_i[i].second << endl;
    // if max(np.absolute(dmse)) < eps: // 종료판정
    float max_dmse = max(abs(dmse.x), abs(dmse.y));
    // cout << max_dmse << endl;
    if (max_dmse < eps)
    {
      break;
    }
  }

  float w0 = w_i[index].first;
  float w1 = w_i[index].second;


  LineComponent line;
  line.w0 = w0;
  line.w1 = w1;
  return line;


}

