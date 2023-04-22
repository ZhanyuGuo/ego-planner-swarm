#ifndef SFM_PLANNER_NODE_H
#define SFM_PLANNER_NODE_H

// C++
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <boost/bind.hpp>

// ROS
#include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Social Force Model
#include <lightsfm/sfm.hpp>

#include "quadrotor_msgs/PositionCommand.h"

class SfmPlanner
{
public:
  SfmPlanner() : first_plan_(true), has_point_cloud_(false)
  {
  }

  ~SfmPlanner()
  {
  }

  void init(ros::NodeHandle& nh);

  void initAgents();

private:
  ros::NodeHandle nh_;

  int agent_number_;
  std::vector<sfm::Agent> agents_;
  int current_agent_id_;

  // std::vector<ros::Subscriber> odom_subs_;
  ros::Timer plan_timer_;

  // void odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id);
  void planCallback(const ros::TimerEvent& e);

  ros::Time current_time_, last_time_;
  bool first_plan_;

  std::vector<ros::Publisher> pos_cmd_pubs_;

  std::vector<ros::Subscriber> point_cloud2_subs_;
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int agent_id);
  std::vector<sensor_msgs::PointCloud> point_clouds_;

  void handleObstacles();
  bool has_point_cloud_;
};

#endif