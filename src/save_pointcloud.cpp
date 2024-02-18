// ROS core
#include <ros/ros.h>

// PCL includes
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_to_pcd", ros::init_options::AnonymousName);

  std::string filename;
  std::string target_frame;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  priv_nh.getParam ("target_frame", target_frame);
  priv_nh.param ("filename", filename, std::string("sample"));

  boost::shared_ptr<sensor_msgs::PointCloud2 const> cloud_msg;
  cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("input", nh, ros::Duration(5.0));

  if(cloud_msg != nullptr) {
    std::stringstream ss;
    if (filename != "") {
      ss << filename << ".pcd";
    }

    pcl::PCDWriter writer;

    if (!target_frame.empty ()) {
      tf::TransformListener tf_listener;
      tf::StampedTransform transform;
      sensor_msgs::PointCloud2 cloud_out;

      try {
        tf_listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(4.0));
        tf_listener.lookupTransform(target_frame, "map", ros::Time(0), transform);
      } catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return (0);
      }

      pcl_ros::transformPointCloud (target_frame, transform, *cloud_msg, cloud_out);

      pcl::PCLPointCloud2 pcl_output;
      pcl_conversions::toPCL(cloud_out, pcl_output);
      writer.writeBinary (ss.str (), pcl_output);
      ROS_INFO ("Data saved to %s", ss.str ().c_str ());
    } else {
      pcl::PCLPointCloud2 pcl_output;
      pcl_conversions::toPCL(*cloud_msg, pcl_output);
      writer.writeBinary (ss.str (), pcl_output);
      ROS_INFO ("Data saved to %s", ss.str ().c_str ());
    }
  } else {
    ROS_ERROR("cannot receive the pcl message");
  }


  return (0);
}
