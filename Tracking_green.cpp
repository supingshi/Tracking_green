#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
// #include <pcl/io/pcdio.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <pcl/impl/point_types.hpp>
#include <math.h>

ros::Publisher pub;
ros::Publisher pub_x;
//ros::Publisher pub_c;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
  pcl::fromPCLPointCloud2(*cloud, cloud_xyzrgb);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_xyzrgb, cloud_filtered, indices);

  int size = cloud_filtered.size();
  //ROS_INFO("%d", size);

  std::vector<float> x_points;
  std::vector<float> y_points;
  std::vector<float> z_points;

  for (int i=0; i<size; i++)
  {
    int r = cloud_filtered.points[i].r;
    int g = cloud_filtered.points[i].g;
    int b = cloud_filtered.points[i].b;

    //ROS_INFO("%d", r);

    //if (r >= 0 and r <= 20 and g >= 0 and g <= 50 and b >= 140 and b <= 200)
    if (r >= 0 and r <= 20 and g >= 0 and g <= 255 and b >= 100 and b <= 255)
    {
      float x = cloud_filtered.points[i].x;
      float y = cloud_filtered.points[i].y;
      float z = cloud_filtered.points[i].z;

      x_points.push_back(x);
      y_points.push_back(y);
      z_points.push_back(z);
    }
  }

  double x_sum = std::accumulate(x_points.begin(), x_points.end(), 0.0);
  double x_mean = x_sum / x_points.size();

  double y_sum = std::accumulate(y_points.begin(), y_points.end(), 0.0);
  double y_mean = y_sum / y_points.size();

  double z_sum = std::accumulate(z_points.begin(), z_points.end(), 0.0);
  double z_mean = z_sum / z_points.size();

  if (!isnan(x_mean))
  {
    ROS_INFO("x: %f", x_mean);
    ROS_INFO("y: %f", y_mean);
    ROS_INFO("z: %f", z_mean);
  }

  /*
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(cloud_filtered, output);
  output = *cloud_msg;
  // Publish the data.
  pub.publish(output);
  //pub_x.publish(x_point);
  //pub_c.publish(cloud);
  */
  //pub_x.publish(width);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_x = nh.advertise<std_msgs::Int16> ("pcl_x", 1);
  //pub_c = nh.advertise<pcl::PCLPointCloud2> ("pcl", 1);

  // Spin
  ros::spin ();
}
API Training Shop Blog About
