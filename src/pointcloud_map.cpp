#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

class PointCloudTo2DMap
{
public:
  PointCloudTo2DMap(ros::NodeHandle nh, ros::NodeHandle nh_private):
    nh_(nh), nh_private_(nh_private)
  {
    nh_private_.param("width", width_, 10.0);
    nh_private_.param("height", height_, 10.0);
    nh_private_.param("resolution", resolution_, 0.05);
    nh_private_.param("min_height", min_height_, -0.1);
    nh_private_.param("max_height", max_height_, 0.1);

    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1,
                                                 boost::bind(&PointCloudTo2DMap::connectCb, this),
                                                  boost::bind(&PointCloudTo2DMap::disconnectCb, this),
                                                  ros::VoidConstPtr(),
                                                  true);

  }
  ~PointCloudTo2DMap(){}

  void connectCb()
  {
    if (pub_.getNumSubscribers() > 0 && sub_.getNumPublishers() == 0)
      {
        ROS_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
        std::string topic_name;
        nh_private_.param("topic_name", topic_name, std::string("point_cloud"));
        sub_ = nh_.subscribe(topic_name, 1, &PointCloudTo2DMap::cloudCb, this);
      }
  }

  void disconnectCb()
  {
    if (pub_.getNumSubscribers() == 0)
      {
        ROS_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
        sub_.shutdown();
      }
  }


  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    nav_msgs::OccupancyGrid output;
    output.header = cloud_msg->header;
    output.info.map_load_time = cloud_msg->header.stamp;
    output.info.resolution = resolution_;
    output.info.width = width_ / resolution_;
    output.info.height = height_ / resolution_;
    // The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
    // with yaw as counterclockwise rotation (yaw=0 means no rotation).
    // Many parts of the system currently ignore yaw.
    // https://www.eureka-moments-blog.com/entry/2022/07/25/170130#Grid-Map%E3%81%AE%E5%88%9D%E6%9C%9F%E5%8C%96
    output.info.origin.position.x = -height_ / 2;
    output.info.origin.position.y = width_ / 2;
    output.info.origin.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
    output.data.resize(output.info.width * output.info.height, 0);


    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"),
           iter_z(*cloud_msg, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
          {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
            continue;
          }

        if (*iter_z > max_height_ || *iter_z < min_height_)
          {
            ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
            continue;
          }

        if (fabs(*iter_x) >= height_ / 2 || fabs(*iter_y) >= width_/2)
          {
            ROS_DEBUG("rejected for out the scope of (%f %f). Point: (%f, %f, %f)", width_, height_, *iter_x, *iter_y, *iter_z);
            continue;
          }

        // TODO, give the correct cell result
        int w = (width_ / 2 - *iter_y) / resolution_;
        int h = (height_ / 2 + *iter_x) / resolution_;
        int index = w + output.info.width * h;
        //ROS_INFO("index: %d", index);
        output.data.at(index) = 100;
      }
    pub_.publish(output);
  }

private:
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double width_, height_, resolution_;
  double min_height_, max_height_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "point_cloud_2_map");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  PointCloudTo2DMap node(nh, nh_private);
  ros::spin();

  return 0;
}
