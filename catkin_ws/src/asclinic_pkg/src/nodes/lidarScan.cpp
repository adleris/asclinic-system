#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include "sensor_msgs/LaserScan.h"
#include <array>

void laserScanSubscriberCallback(const sensor_msgs::LaserScan& msg);

// Respond to subscriber receiving a message
void laserScanSubscriberCallback(const sensor_msgs::LaserScan& msg)
{
  ROS_INFO_STREAM("Message received with angle_min = " << msg.angle_min << " [rad], angle_max = " << msg.angle_max << " [rad], range_min = " << msg.range_min << " [m], range_max = " << msg.range_max << " [m]");

  // TODO: ADD PARSING
  // Now process the msg.ranges data to
  // interpret the robot's surroundings
}

int main(int argc, char **argv){

    ros::init(argc, argv, "subscriber_cpp_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("[PREPARING LIDAR SUBSCRIBER NODE]: " << nh.getNamespace());
    std::string ns_for_group = ros::this_node::getNamespace();

    // Initialise a node handle to the group namespace
    ros::NodeHandle nh_for_asc_group("/asc");

    // Initialise a subscriber to the RPLidar scan
    uint32_t queue_size = 10;
    ros::Subscriber rplidar_scan_subscriber = nh_for_asc_group.subscribe("scan", queue_size, laserScanSubscriberCallback);

    // keep this node running
    ros::spin();

    return 0;
}
