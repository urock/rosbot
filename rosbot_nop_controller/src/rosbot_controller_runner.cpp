#include "rosbot_controller.hpp"

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <cmath>
#include <iostream>

constexpr size_t ground_model_id = 0;
constexpr size_t rosbot_model_id = 1;
constexpr float dt = 0.1;
constexpr float epsterm = 0.05;
constexpr float k = 0.025;
constexpr float b = 0.21 * 2;

Model::State rosbot_state{};
Model::State rosbot_goal{};


void target_sub_cb(const geometry_msgs::PointStamped::ConstPtr &msg) 
{
  rosbot_goal.x = msg->point.x;
  rosbot_goal.y = msg->point.y;
  rosbot_goal.yaw =
      atan2(rosbot_goal.y - rosbot_state.y, rosbot_goal.x - rosbot_state.x);
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

int main(int argc, char **argv) 
{

  NetOper netOp;
  netOp.setNodesForVars({0, 1, 2});   // Pnum
  netOp.setNodesForParams({3, 4, 5}); // Rnum
  netOp.setNodesForOutput({22, 23});  // Dnum
  netOp.setCs(qc);                    // set Cs
  netOp.setPsi(NopPsiN);

  Controller nop_controller(netOp, rosbot_goal, rosbot_state);

  ros::init(argc, argv, "rosbot_nop_controller");
  ros::NodeHandle node;

  // subscribers
  ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
  ros::Subscriber target_sub = node.subscribe("clicked_point", 5, target_sub_cb);
  // publishers
  ros::Publisher control_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // main loop
  ros::Rate rate(1. / dt);
  while (ros::ok()) {
    ros::spinOnce();

    nop_controller.setGoal(rosbot_state, rosbot_goal);

    geometry_msgs::Twist ctrl;
    if (rosbot_state.dist(rosbot_goal) > epsterm) 
    {
      // get control with NOP
      const Model::Control& u = nop_controller.calcControl(rosbot_state);
      ctrl.linear.x = k * 0.5 * (u.left + u.right);
      ctrl.angular.z = k * (u.left - u.right) / b;

      // get control from proportional controller
      // const Model::Control& u = nop_controller.calcPropControl(rosbot_state);
      // ctrl.linear.x = u.left;
      // ctrl.angular.z = u.right;
    }
    control_pub.publish(ctrl);

    ROS_INFO("State: %lf %lf %lf \n", rosbot_state.x, rosbot_state.y, rosbot_state.yaw);
    ROS_INFO("Target: %lf %lf %lf\n", rosbot_goal.x, rosbot_goal.y, rosbot_goal.yaw);
    ROS_INFO("Control [cmd_vel]: %lf %lf\n", ctrl.linear.x, ctrl.angular.z);

    rate.sleep();

  }

  return 0;
}