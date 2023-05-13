#include <sfm_planner/sfm_planner_node.h>

void SfmPlanner::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("sfm/agent_number", agent_number_, -1);
  nh_.param("sfm/agent_id", agent_id_, -1);

  pos_cmd_pub_ =
      nh_.advertise<quadrotor_msgs::PositionCommand>("/drone_" + std::to_string(agent_id_) + "_planning/pos_cmd", 1);

  point_cloud2_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      "/drone_" + std::to_string(agent_id_) + "_pcl_render_node/cloud", 1, &SfmPlanner::pointcloudCallback, this);

  others_.resize(agent_number_);
  other_odoms_.resize(agent_number_);
  for (int i = 0; i < agent_number_; i++)
  {
    // get other agents' positions
    ros::Subscriber odom_sub =
        nh_.subscribe<nav_msgs::Odometry>("/drone_" + std::to_string(i) + "_visual_slam/odom", 1,
                                          boost::bind(&SfmPlanner::odometryCallback, this, _1, i));
    odom_subs_.push_back(odom_sub);
  }

  initAgent();

  plan_timer_ = nh.createTimer(ros::Duration(0.01), &SfmPlanner::planCallback, this);
}

void SfmPlanner::initAgent()
{
  agent_.id = agent_id_;

  // cycle
  nh_.param("sfm/agent_cycle", agent_.cyclicGoals, false);

  // queue size
  nh_.param("sfm/queue_size", queue_size_, 500);

  // init position and velocity
  double init_x, init_y;
  nh_.param("sfm/agent_init_x", init_x, 0.0);
  nh_.param("sfm/agent_init_y", init_y, 0.0);
  agent_.position.set(init_x, init_y);
  agent_.yaw = utils::Angle::fromRadian(0.0);
  agent_.velocity.set(0.0, 0.0);
  agent_.linearVelocity = 0.0;
  agent_.angularVelocity = 0.0;
  agent_.radius = 0.5;

  // waypoints
  int waypoint_number;
  nh_.param("sfm/agent_waypoint_number", waypoint_number, 0);
  for (int j = 0; j < waypoint_number; j++)
  {
    double x, y;
    nh_.param("sfm/agent_waypoint" + std::to_string(j) + "_x", x, 0.0);
    nh_.param("sfm/agent_waypoint" + std::to_string(j) + "_y", y, 0.0);

    sfm::Goal waypoint;
    waypoint.center.set(x, y);
    waypoint.radius = 0.3;
    agent_.goals.push_back(waypoint);
  }

  // max velocity
  nh_.param("sfm/agent_max_velocity", agent_.desiredVelocity, 0.8);

  // weights
  nh_.param("sfm/agent_goal_weight", agent_.params.forceFactorDesired, 2.0);
  nh_.param("sfm/agent_obstacle_weight", agent_.params.forceFactorObstacle, 10.0);
  nh_.param("sfm/agent_social_weight", agent_.params.forceFactorSocial, 2.1);
  nh_.param("sfm/agent_sigma_obstacle", agent_.params.forceSigmaObstacle, 0.2);

  // group weights TODO

  // INFO
  // ROS_INFO("Agent %d, position: (%.2f, %.2f)", agent_id_, agent_.position.getX(), agent_.position.getY());
  // int j = 0;
  // for (auto waypoint : agent_.goals)
  // {
  //   ROS_INFO("Agent %d, goal%d: (%.2f, %.2f)", agent_id_, j, waypoint.center.getX(), waypoint.center.getY());
  //   j++;
  // }
}

void SfmPlanner::odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id)
{
  if (!odom_flag_)
    odom_flag_ = true;

  other_odoms_[agent_id] = *msg;
}

void SfmPlanner::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (msg->data.size() == 0)
  {
    point_cloud_flag_ == false;
    return;
  }

  if (!point_cloud_flag_)
    point_cloud_flag_ = true;

  sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud_);
}

void SfmPlanner::planCallback(const ros::TimerEvent& e)
{
  if (!plan_flag_)
  {
    last_time_ = ros::Time::now();
    plan_flag_ = true;
    return;
  }

  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  last_time_ = current_time_;

  // update closest obstacle
  handleObstacles();

  // update pedestrian around
  handlePedestrians();

  // Compute Social Forces
  sfm::SFM.computeForces(agent_, others_);

  // Update model
  sfm::SFM.updatePosition(agent_, dt);

  // output position, velocity, etc.
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = current_time_;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = 0;

  cmd.position.x = agent_.position.getX();
  cmd.position.y = agent_.position.getY();
  cmd.position.z = 1.0;

  cmd.velocity.x = agent_.velocity.getX();
  cmd.velocity.y = agent_.velocity.getY();
  cmd.velocity.z = 0.0;

  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;

  cmd.yaw = agent_.yaw.toRadian();
  cmd.yaw_dot = agent_.angularVelocity;

  pos_cmd_pub_.publish(cmd);
}

void SfmPlanner::handleObstacles()
{
  if (!point_cloud_flag_ || !odom_flag_)
    return;

  // TODO, can be improved to queue
  if (agent_.obstacles1.size() > queue_size_)
    agent_.obstacles1.erase(agent_.obstacles1.begin());

  double x = agent_.position.getX();
  double y = agent_.position.getY();
  double z = 1.0;

  double min_x, min_y, min_dist = 1000.0;
  for (int j = 0; j < point_cloud_.points.size(); j++)
  {
    // only consider 2D plane
    if (std::fabs(point_cloud_.points[j].z - z) < 0.1)
      continue;

    double x_j = point_cloud_.points[j].x;
    double y_j = point_cloud_.points[j].y;
    double dist = std::hypot(x - x_j, y - y_j);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_x = x_j;
      min_y = y_j;
    }
  }
  utils::Vector2d ob(min_x, min_y);
  agent_.obstacles1.push_back(ob);

  // TODO, maybe agents seen as obstacles...
  // agent_.obstacles2.clear();
  // for (int i = 0; i < agent_number_; i++)
  // {
  //   if (i == agent_id_)
  //     continue;

  //   utils::Vector2d ag(other_odoms_[i].pose.pose.position.x, other_odoms_[i].pose.pose.position.y);
  //   agent_.obstacles2.push_back(ag);
  // }
}

void SfmPlanner::handlePedestrians()
{
  if (!odom_flag_)
    return;

  for (int i = 0; i < agent_number_; i++)
  {
    if (i == agent_id_)
      continue;

    sfm::Agent& other = others_[i];
    other.id = i;
    other.position.set(other_odoms_[i].pose.pose.position.x, other_odoms_[i].pose.pose.position.y);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(other_odoms_[i].pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    other.yaw = utils::Angle::fromRadian(yaw);
    other.radius = agent_.radius;
    other.velocity.set(other_odoms_[i].twist.twist.linear.x, other_odoms_[i].twist.twist.linear.y);
    other.linearVelocity = other.velocity.norm();
    other.angularVelocity = other_odoms_[i].twist.twist.angular.z;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sfm_planner_node");
  ros::NodeHandle nh("~");

  SfmPlanner sfm_planner;
  sfm_planner.init(nh);

  ros::spin();

  return 0;
}
