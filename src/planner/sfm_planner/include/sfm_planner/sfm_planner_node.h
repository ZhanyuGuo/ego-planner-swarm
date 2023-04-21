#ifndef SFM_PLANNER_NODE_H
#define SFM_PLANNER_NODE_H

// C++
#include <iostream>
#include <vector>
#include <string>
// #include <boost/bind.hpp>

// ROS
#include <ros/ros.h>
// #include <nav_msgs/Odometry.h>

// Social Force Model
#include <lightsfm/sfm.hpp>

#include "quadrotor_msgs/PositionCommand.h"

class SfmPlanner
{
public:
  SfmPlanner() : first_plan_(true)
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

  void handleObstacles();
};

#endif