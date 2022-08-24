#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "controller.hpp"
#include <cmath>
#include <iostream>

constexpr size_t ground_model_id = 0;
constexpr size_t rosbot_model_id = 1;
constexpr float dt = 0.1;
constexpr float epsterm = 0.15;
constexpr float k = 0.01;
constexpr float b = 0.21 * 2;

Model::State rosbot_state{};
Model::State rosbot_goal{};
bool on_new_goal = false;

void target_sub_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
  rosbot_goal.x = msg->point.x;
  rosbot_goal.y = msg->point.y;
  rosbot_goal.yaw =
      atan2(rosbot_goal.y - rosbot_state.y, rosbot_goal.x - rosbot_state.x);
  on_new_goal = true;

  ROS_INFO("State: %lf %lf %lf\n", rosbot_state.x, rosbot_state.y, rosbot_state.yaw*(180. / M_PI));

  ROS_INFO("Target: %lf %lf %lf\n", rosbot_goal.x, rosbot_goal.y, rosbot_goal.yaw*(180. / M_PI));


}

void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {

  const auto &q = msg->pose[rosbot_model_id].orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  rosbot_state.x = msg->pose[rosbot_model_id].position.x;
  rosbot_state.y = msg->pose[rosbot_model_id].position.y;
  rosbot_state.yaw = yaw;
}

int main(int argc, char **argv) {

  NetOper netOp;
  netOp.setNodesForVars({0, 1, 2});   // Pnum
  netOp.setNodesForParams({3, 4, 5}); // Rnum
  netOp.setNodesForOutput({22, 23});  // Dnum
  netOp.setCs(qc);                    // set Cs
  netOp.setPsi(NopPsiN);

  Model::Control u{};

  Controller nop_controller(rosbot_goal, netOp);

  ros::init(argc, argv, "test_node");

  ros::NodeHandle n;
  ros::Rate loop_rate(1. / dt);

  geometry_msgs::Twist cmd_vel;

  ros::Subscriber models_sub =
      n.subscribe("gazebo/model_states", 5, model_state_cb);
  ros::Subscriber target_sub = n.subscribe("clicked_point", 5, target_sub_cb);

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  while (ros::ok()) {
    ros::spinOnce();

    nop_controller.setGoal(rosbot_goal);

    if (rosbot_state.dist(rosbot_goal) > epsterm) {
      u = nop_controller.calcControl(rosbot_state);
      cmd_vel.linear.x = k * 0.5 * (u.left + u.right);
      cmd_vel.angular.z = k * (u.left - u.right) / b;
    } 
    else 
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }
    cmd_vel_pub.publish(cmd_vel);

    // ROS_INFO("U: %lf %lf\n", u.left, u.right);
    // double yaw_deg = rosbot_state.yaw * (180. / pi);
    // ROS_INFO("yaw: %lf\n", yaw_deg);

    // ROS_INFO("State: %lf %lf %lf\n", rosbot_state.x, rosbot_state.y, rosbot_state.yaw*(180. / pi));

    loop_rate.sleep();
  }

  return 0;
}