#ifndef CLaserOdometry2DNode_H
#define CLaserOdometry2DNode_H

#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace rf2o {

/**
 * The ROS 2 wrapper around the rf2o algorithm.
 *
 * This is the ONLY rclcpp::Node of this package. The algorithm itself (CLaserOdometry2D) is a
 * plain class that receives this node's logger and clock through its constructor, so running
 * this package registers exactly one node in the ROS graph.
 * See: https://github.com/MAPIRlab/rf2o_laser_odometry/issues/42
 */
class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  void process();
  void publish();
  bool setLaserPoseFromTf();
  bool scan_available();

  // Params & vars
  /// The odometry algorithm. Not a node: it is built with this node's logger and clock.
  std::unique_ptr<CLaserOdometry2D> rf2o_ref;
  bool                publish_tf, new_scan_available;
  double              freq;
  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  sensor_msgs::msg::LaserScan                     last_scan;
  bool                                            GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer>                buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster>  odom_broadcaster;
  nav_msgs::msg::Odometry                         initial_robot_pose;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub;

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
};

} /* namespace rf2o */

#endif // CLaserOdometry2DNode_H
