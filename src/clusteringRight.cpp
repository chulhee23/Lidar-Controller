#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>


ros::Publisher pub;


void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
pcl::PointCloud<pcl::PointXYZ> inputCloud;
pcl::fromROSMsg(*msg, inputCloud);

pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
kdtree->setInputCloud (inputCloud.makeShared());

std::vector<pcl::PointIndices> clusterIndices; 

pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance (0.6); // set distance threshold = 0.9m

ec.setMinClusterSize (2); // set Minimum Cluster Size
ec.setMaxClusterSize (100); // set Maximum Cluster Size
ec.setSearchMethod (kdtree);
ec.setInputCloud (inputCloud.makeShared());
ec.extract (clusterIndices);


std::vector<pcl::PointIndices>::const_iterator it;

for (it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
{
  pcl::PointCloud<pcl::PointXYZ> filteredCloud;

  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    filteredCloud.push_back (inputCloud[*pit]);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(filteredCloud, output);
  output.header.frame_id = "/laser";

  pub.publish(output);
  }

}

int main(int argc, char** argv)
{
ros::init(argc, argv, "clusteringRight");
ros::NodeHandle nh;

// << Subscribe Topic >>
// topic name : /filterRight
// topic type : sensor_msgs::PointCloud2
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filterRight", 1, callbackFcn);

// << Publish Topic >>
// topic name : /clusterRight
// topic type : sensor_msgs::PointCloud2
pub = nh.advertise<sensor_msgs::PointCloud2>("/clusterRight", 1);

ros::spin();
return 0;
}

