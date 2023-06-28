#include "rosbot_nop_stabilization.hpp"


RosbotNOPStabilization::RosbotNOPStabilization(ros::NodeHandle& nh, NetOper& netOp)
{
  models_sub = nh.subscribe("gazebo/model_states", 5, &RosbotNOPStabilization::model_state_cb, this);
  target_sub = nh.subscribe("clicked_point", 5, &RosbotNOPStabilization::target_sub_cb, this);
  control_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  run(netOp);
}


void RosbotNOPStabilization::target_sub_cb(const geometry_msgs::PointStamped::ConstPtr &msg) 
{

  rosbot_goal.x = msg->point.x;
  rosbot_goal.y = msg->point.y;
  rosbot_goal.yaw =
      atan2(rosbot_goal.y - rosbot_state.y, rosbot_goal.x - rosbot_state.x);

  ROS_INFO("Target: %lf %lf %lf\n", rosbot_goal.x, rosbot_goal.y, rosbot_goal.yaw);
}

void RosbotNOPStabilization::update_target_yaw()
{
  rosbot_goal.yaw =
      atan2(rosbot_goal.y - rosbot_state.y, rosbot_goal.x - rosbot_state.x);
}

void RosbotNOPStabilization::model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {

  const auto &q = msg->pose[rosbot_model_id].orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  rosbot_state.x = msg->pose[rosbot_model_id].position.x;
  rosbot_state.y = msg->pose[rosbot_model_id].position.y;
  rosbot_state.yaw = yaw;
}

void RosbotNOPStabilization::run(NetOper& netOp)
{
  // main loop
  RosbotNOPController nop_controller(rosbot_goal, netOp);
  ros::Rate rate(1. / dt);
  while (ros::ok()) {
    ros::spinOnce();

    // update_target_yaw();
    nop_controller.setGoal(rosbot_goal);

    geometry_msgs::Twist ctrl;
    if (rosbot_state.distXY(rosbot_goal) > eps) 
    {
      const Model::Control& u = nop_controller.calcControl(rosbot_state);
      ctrl.linear.x = u.left;
      ctrl.angular.z = u.right;

    }
    control_pub.publish(ctrl);

    ROS_INFO("State: %lf %lf %lf \n", rosbot_state.x, rosbot_state.y, rosbot_state.yaw);
    ROS_INFO("Target: %lf %lf %lf\n", rosbot_goal.x, rosbot_goal.y, rosbot_goal.yaw);
    ROS_INFO("Control [cmd_vel]: %lf %lf\n", ctrl.linear.x, ctrl.angular.z);

    rate.sleep();

  }
}



int main(int argc, char **argv) 
{

  NetOper netOp;
  netOp.setLocalTestsParameters();

  ros::init(argc, argv, "rosbot_nop_stabilization");
  ros::NodeHandle nh;
  RosbotNOPStabilization(nh, netOp);


  return 0;
}