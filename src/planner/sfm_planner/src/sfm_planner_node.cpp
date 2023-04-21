
#include <sfm_planner/sfm_planner_node.h>

void SfmPlanner::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("sfm/agent_number", agent_number_, 0);
  // ROS_INFO("Number of agents: %d", agent_number_);

  // pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  for (int i = 0; i < agent_number_; i++)
  {
    ros::Publisher pos_cmd_pub =
        nh_.advertise<quadrotor_msgs::PositionCommand>("/drone_" + std::to_string(i) + "_planning/pos_cmd", 1);
    pos_cmd_pubs_.push_back(pos_cmd_pub);
  }

  // for (int i = 0; i < agent_number_; i++)
  // {
  //   ros::Subscriber odom_sub =
  //       nh_.subscribe<nav_msgs::Odometry>("/drone_" + std::to_string(i) + "_visual_slam/odom", 1,
  //                                         boost::bind(&SfmPlanner::odometryCallback, this, _1, i));
  //   odom_subs_.push_back(odom_sub);
  // }

  initAgents();

  plan_timer_ = nh.createTimer(ros::Duration(0.1), &SfmPlanner::planCallback, this);
}

void SfmPlanner::initAgents()
{
  for (int i = 0; i < agent_number_; i++)
  {
    sfm::Agent agent;

    // id
    agent.id = i;

    // cycle
    nh_.param("sfm/agent" + std::to_string(i) + "_cycle", agent.cyclicGoals, false);
    // ROS_INFO("Agent %d, cycle: %d", i, agent.cyclicGoals);

    // init position and velocity
    double init_x, init_y;
    nh_.param("sfm/agent" + std::to_string(i) + "_init_x", init_x, 0.0);
    nh_.param("sfm/agent" + std::to_string(i) + "_init_y", init_y, 0.0);
    agent.position.set(init_x, init_y);
    agent.yaw = utils::Angle::fromRadian(0.0);
    agent.velocity.set(0.0, 0.0);
    agent.linearVelocity = 0.0;
    agent.angularVelocity = 0.0;

    // waypoints
    int waypoint_number;
    nh_.param("sfm/agent" + std::to_string(i) + "_waypoint_number", waypoint_number, 0);
    for (int j = 0; j < waypoint_number; j++)
    {
      sfm::Goal waypoint;
      double x, y;
      nh_.param("sfm/agent" + std::to_string(i) + "_waypoint" + std::to_string(j) + "_x", x, 0.0);
      nh_.param("sfm/agent" + std::to_string(i) + "_waypoint" + std::to_string(j) + "_y", y, 0.0);
      waypoint.center.set(x, y);
      waypoint.radius = 0.3;
      agent.goals.push_back(waypoint);
    }

    // max velocity
    nh_.param("sfm/agent" + std::to_string(i) + "_max_velocity", agent.desiredVelocity, 0.8);

    // weights
    nh_.param("sfm/agent" + std::to_string(i) + "_goal_weight", agent.params.forceFactorDesired, 2.0);
    nh_.param("sfm/agent" + std::to_string(i) + "_obstacle_weight", agent.params.forceFactorObstacle, 10.0);
    nh_.param("sfm/agent" + std::to_string(i) + "_social_weight", agent.params.forceFactorSocial, 2.1);

    // group weights
    // TODO

    // add agent
    agents_.push_back(agent);
  }

  // for (int i = 0; i < agent_number_; i++)
  // {
  //   sfm::Agent agent = agents_[i];
  //   ROS_INFO("Agent %d, position: (%.2f, %.2f)", i, agent.position.getX(), agent.position.getY());

  //   int j = 0;
  //   for (auto waypoint : agent.goals)
  //   {
  //     ROS_INFO("Agent %d, goal%d: (%.2f, %.2f)", i, j, waypoint.center.getX(), waypoint.center.getY());
  //     j++;
  //   }
  // }
}

// void SfmPlanner::odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id)
// {
//   std::cout << agent_id << std::endl;
// }

void SfmPlanner::planCallback(const ros::TimerEvent& e)
{
  if (first_plan_)
  {
    last_time_ = ros::Time::now();
    first_plan_ = false;
    return;
  }

  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  last_time_ = current_time_;

  // // update closest obstacle
  // handleObstacles();

  // // update pedestrian around
  // handlePedestrians();

  // Compute Social Forces
  agents_ = sfm::SFM.computeForces(agents_);

  // Update model
  agents_ = sfm::SFM.updatePosition(agents_, dt);

  // output position, velocity, etc. TODO
  for (int i = 0; i < agent_number_; i++)
  {
    sfm::Agent agent = agents_[i];
    // ROS_INFO("Agent %d, position: (%.2f, %.2f)", i, agent.position.getX(), agent.position.getY());

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = current_time_;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;

    cmd.position.x = agent.position.getX();
    cmd.position.y = agent.position.getY();
    cmd.position.z = 1.0;

    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;

    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;

    cmd.yaw = agent.yaw.toRadian();
    cmd.yaw_dot = 0.0;

    pos_cmd_pubs_[i].publish(cmd);
  }
}

int main(int argc, char** argv)
{
  // # a simple way to valid sfm-model
  // 1. disable ego_planner
  // 2. enable sfm_planner
  // 3. publish quadrotor_msgs::PositionCommand with update x, y, z to /position_cmd
  // 4. centerlized control first: all position known, all controllable...
  // 5. global plan first: no feedback from environment...
  ros::init(argc, argv, "sfm_planner_node");

  ros::NodeHandle nh("~");

  SfmPlanner sfm_planner;
  sfm_planner.init(nh);

  ros::spin();

  return 0;
}
