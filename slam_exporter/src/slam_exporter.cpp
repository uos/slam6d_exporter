#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "globals.icc"
#include <pcl_ros/transforms.h>

#include <fstream>
using std::ofstream;
#include <iostream>
using std::cout;
using std::endl;

tf::TransformListener *tl_;

bool needRequest_;
bool requested_;
bool use_point_cloud2_;
std::string fixed_frame_;
std::string robot_frame_;
std::string pc_topic_;
std::string request_topic_;

bool getTransform(double *t, double *ti, double *rP, double *rPT, tf::TransformListener *listener, ros::Time time)
{
  tf::StampedTransform transform;

  std::string error_msg;
  bool success = listener->waitForTransform(fixed_frame_, robot_frame_, time, ros::Duration(3.0), ros::Duration(0.01), &error_msg);

  if (!success)
  {
    ROS_WARN("Could not get transform, ignoring point cloud! %s", error_msg.c_str());
    return false;
  }

  listener->lookupTransform(fixed_frame_, robot_frame_, time, transform);

  double mat[9];
  double x = transform.getOrigin().getX() * 100;
  double y = transform.getOrigin().getY() * 100;
  double z = transform.getOrigin().getZ() * 100;
  mat[0] = transform.getBasis().getRow(0).getX();
  mat[1] = transform.getBasis().getRow(0).getY();
  mat[2] = transform.getBasis().getRow(0).getZ();

  mat[3] = transform.getBasis().getRow(1).getX();
  mat[4] = transform.getBasis().getRow(1).getY();
  mat[5] = transform.getBasis().getRow(1).getZ();

  mat[6] = transform.getBasis().getRow(2).getX();
  mat[7] = transform.getBasis().getRow(2).getY();
  mat[8] = transform.getBasis().getRow(2).getZ();

  t[0] = mat[4];
  t[1] = -mat[7];
  t[2] = -mat[1];
  t[3] = 0.0;

  t[4] = -mat[5];
  t[5] = mat[8];
  t[6] = mat[2];
  t[7] = 0.0;

  t[8] = -mat[3];
  t[9] = mat[6];
  t[10] = mat[0];
  t[11] = 0.0;

  // translation
  t[12] = -y;
  t[13] = z;
  t[14] = x;
  t[15] = 1;
  M4inv(t, ti);
  Matrix4ToEuler(t, rPT, rP);

  return true;
}

void writePose(int j, double rP[3], double rPT[3])
{
  char pose_str[13];
  sprintf(pose_str, "scan%03d.pose", j);
  ofstream pose(pose_str);
  pose << rP[0] << " " << rP[1] << " " << rP[2] << endl << deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]);
  pose.close();
}

void reqCallback(const std_msgs::String::ConstPtr& e)
{
  ROS_INFO_STREAM("Request received: " << e->data);
  requested_ = true;
}

void pcCallback(const sensor_msgs::PointCloud::ConstPtr& untransformed_cloud)
{
  //ignore first scan (tf can't transform it and its incomplete anyway)
  static bool first = true;
  if (first)
  {
    first = false;
    return;
  }

  if (needRequest_ && !requested_)
  {
    return;
  }

  static int j = 0;

  // ----- write transform fixed_frame --> robot_frame to pose file
  double t[16], ti[16], rP[3], rPT[3];
  bool success = getTransform(t, ti, rP, rPT, tl_, untransformed_cloud->header.stamp);
  if (!success)
    return;

  writePose(j, rP, rPT);

  // ----- transform point cloud from sensor frame to robot_frame
  boost::shared_ptr<sensor_msgs::PointCloud> cloud(new sensor_msgs::PointCloud);
  tl_->transformPointCloud(robot_frame_, *untransformed_cloud, *cloud);

  if (!success)
    return;

  char scan_str[11];
  sprintf(scan_str, "scan%03d.3d", j++);
  ofstream scan(scan_str);

  size_t i;
  double p[3];
  for (i = 0; i < cloud->points.size(); i++)
  {
    p[0] = cloud->points[i].y * -100;
    p[1] = cloud->points[i].z * 100;
    p[2] = cloud->points[i].x * 100;

    // transform3(ti, p);

    if (!isnan(p[0]) && !isnan(p[1]) && !isnan(p[2]))
    {
      scan << p[0] << " " << p[1] << " " << p[2] << endl;
    }
  }
  scan.close();
  ROS_INFO("wrote %zu points to file %s (backlog: %f s)", i, scan_str, (ros::Time::now() - cloud->header.stamp).toSec());

  requested_ = false;
}

inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
{
  for (size_t i = 0; i < cloud->fields.size(); ++i)
  {
    if (cloud->fields[i].name == channel)
    {
      return i;
    }
  }

  return -1;
}

void pc2aCallback(const sensor_msgs::PointCloud2Ptr& untransformed_cloud)
{
  static int j = 0;

  // ----- write transform fixed_frame --> robot_frame to pose file
  double t[16], ti[16], rP[3], rPT[3];
  bool success = getTransform(t, ti, rP, rPT, tl_, untransformed_cloud->header.stamp);
  if (!success)
    return;

  writePose(j, rP, rPT);

  // ----- transform point cloud from sensor frame to robot_frame
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud(new sensor_msgs::PointCloud2);
  success = pcl_ros::transformPointCloud(robot_frame_, *untransformed_cloud, *cloud, *tl_);
  if (!success)
    return;

  char scan_str[11];
  sprintf(scan_str, "scan%03d.3d", j++);
  ofstream scan(scan_str);

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");
  int32_t rgbi = findChannelIndex(cloud, "rgb");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return;
  }

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  uint32_t rgboff = -1;
  if (rgbi != -1)
    rgboff = cloud->fields[rgbi].offset;
  const uint32_t point_step = cloud->point_step;
  const size_t point_count = cloud->width * cloud->height;

  if (point_count == 0)
  {
    return;
  }

  const uint8_t* ptr = &cloud->data.front();
  size_t i;
  double p[3];
  for (i = 0; i < point_count; ++i)
  {
    p[0] = *reinterpret_cast<const float*>(ptr + yoff) * -100;
    p[1] = *reinterpret_cast<const float*>(ptr + zoff) * 100;
    p[2] = *reinterpret_cast<const float*>(ptr + xoff) * 100;

    // transform3(ti, p);

    if (!isnan(p[0]) && !isnan(p[1]) && !isnan(p[2]))
    {
      scan << p[0] << " " << p[1] << " " << p[2];

      if (rgbi != -1)
      {
        uint32_t rgb = *reinterpret_cast<const uint32_t*>(ptr + rgboff);
        int r = ((rgb >> 16) & 0xff);
        int g = ((rgb >> 8) & 0xff);
        int b = (rgb & 0xff);

        scan << " " << r << " " << g << " " << b;
      }
      scan << endl;
    }

    ptr += point_step;
  }

  scan.close();
  ROS_INFO("wrote %zu points to file %s (backlog: %f s)", i, scan_str, (ros::Time::now() - cloud->header.stamp).toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_exporter");

  // Only dump points to file when request received.
  if (argc > 1 && strcmp(argv[1], "--withrequest") == 0)
  {
    ROS_INFO("Scan will only be exported when requested as defined by parameter.");
    needRequest_ = true;
  }
  else
    needRequest_ = false;

  requested_ = false;

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  tl_ = new tf::TransformListener(ros::Duration(60.0));

  pn.param("fixed_frame", fixed_frame_, std::string("odom_combined"));
  pn.param("robot_frame", robot_frame_, std::string("base_link"));
  pn.param("pc_topic", pc_topic_, std::string("/points_in"));
  pn.param("request_topic", request_topic_, std::string("/request"));
  pn.param("use_point_cloud2", use_point_cloud2_, true);

  ros::Subscriber sub; // must be declared outside of "if" scope
  if (use_point_cloud2_)
    sub = n.subscribe(pc_topic_, 100, pc2aCallback);
  else
    sub = n.subscribe(pc_topic_, 100, pcCallback);

  ros::Subscriber scanRequest = n.subscribe(request_topic_, 1, reqCallback);

  ROS_INFO("slam_exporter initialized with fixed_frame = \"%s\", robot_frame = \"%s\", ", fixed_frame_.c_str(), robot_frame_.c_str());
  ros::spin();
  return 0;
}
