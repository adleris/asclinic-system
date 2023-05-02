#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include <array>

class LidarParserNode {
public:
  LidarParserNode() : nh_("~"), nh_for_asc_group_("/asc") {
    ROS_INFO_STREAM("[PREPARING LIDAR SUBSCRIBER NODE]: " << nh_.getNamespace());
    obstacle_detection_pub_ = nh_for_asc_group_.advertise<std_msgs::Bool>("sensors/obstacle_scan", queue_size_);
    rplidar_scan_subscriber_ = nh_for_asc_group_.subscribe("scan", queue_size_, &LidarParserNode::laserScanSubscriberCallback, this);
  }

  void laserScanSubscriberCallback(const sensor_msgs::LaserScan& msg) {
    parseScan(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_for_asc_group_;
  ros::Subscriber rplidar_scan_subscriber_;
  ros::Publisher obstacle_detection_pub_;
  uint32_t queue_size_ = 10;

  void parseScan(const sensor_msgs::LaserScan& msg) {
    // parse a scan and see if there are any obstacles around us
    for (auto i : msg.ranges) {
      if (i < 0.4) {
        // ROS_INFO_STREAM("TOO CLOSE (" << i << ")");
        std_msgs::Bool msg;
        msg.data = true;
        obstacle_detection_pub_.publish(msg);
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber_cpp_node");

  LidarParserNode lidar_subscriber_node;

  // keep this node running
  ros::spin();

  return 0;
}
