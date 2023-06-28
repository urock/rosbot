#include "rosbot_controller.hpp"

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <cmath>
#include <iostream>


class RosbotNOPStabilization
{

    // NetOper netOp;
    // ros::NodeHandle nh;
    // RosbotNOPController nop_controller;

    ros::Subscriber models_sub;
    ros::Subscriber target_sub;
    ros::Publisher control_pub;
    
    Model::State rosbot_state{};
    Model::State rosbot_goal{};

    const size_t rosbot_model_id = 1;
    const float dt = 0.1; // [sec]
    const float eps = 0.1; // [m]

    void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void target_sub_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
    void update_target_yaw();

public:
    RosbotNOPStabilization();

    RosbotNOPStabilization(ros::NodeHandle& nh, NetOper& netOp);
    void run(NetOper& netOp);

};