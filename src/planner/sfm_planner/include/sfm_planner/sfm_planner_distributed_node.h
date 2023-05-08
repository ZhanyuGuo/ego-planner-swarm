#ifndef SFM_PLANNER_DISTRIBUTED_NODE_H
#define SFM_PLANNER_DISTRIBUTED_NODE_H

// C++
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <boost/bind.hpp>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>

// Social Force Model
#include <lightsfm/sfm.hpp>

#include "quadrotor_msgs/PositionCommand.h"

class SfmPlanner
{
public:
  SfmPlanner() : plan_flag_(false), point_cloud_flag_(false), odom_flag_(false)
  {
  }

  ~SfmPlanner()
  {
  }

  void init(ros::NodeHandle& nh);

  void initAgent();

private:
  bool plan_flag_, point_cloud_flag_, odom_flag_;
  int agent_number_, agent_id_;
  int queue_size_;
  sfm::Agent agent_;
  std::vector<sfm::Agent> others_;
  std::vector<nav_msgs::Odometry> other_odoms_;
  // std::vector<utils::Vector2d> others_p_;

  ros::NodeHandle nh_;
  ros::Timer plan_timer_;
  ros::Time current_time_, last_time_;

  ros::Publisher pos_cmd_pub_;

  std::vector<ros::Subscriber> odom_subs_;
  ros::Subscriber point_cloud2_sub_;

  sensor_msgs::PointCloud point_cloud_;

  void handleObstacles();
  void handlePedestrians();

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void planCallback(const ros::TimerEvent& e);
};

#endif