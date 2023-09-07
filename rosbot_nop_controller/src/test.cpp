#include <iostream>
#include <vector>
#include <limits>
#include <float.h>
#include <random>
#include <cmath>

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "PSO.hpp"
#include "rosbot_controller.hpp"



std::vector<float> runPSO(Model::State main_goal)
{
    float time_step = 2; // sec
    float dt = 0.01; // sec
    size_t numParticles = 20;
    size_t maxIter = 10;
    std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)
    
    auto pso = PSO(q, numParticles, maxIter, main_goal, time_step, dt);

    Model::State currState = {0., 0., 0.};
    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
        run_to_goal(currState, Goal, dt, time_step);
    }
	return pso.best_global_state;

}

////// PSO OPTIMIZATION



constexpr size_t rosbot_model_id = 1;
constexpr float dt = 0.01;

Model::State rosbot_state{};
Model::State rosbot_goal{};


void update_target_yaw()
{
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

    constexpr float  MAX_TIME = 6.;
    constexpr float TIME_STEP = 2.;
    
    NetOper nop = NetOper();
    nop.setLocalTestsParameters();

    RosbotNOPController nop_controller(rosbot_goal, nop);

    ros::init(argc, argv, "rosbot_nop_controller");
    ros::NodeHandle node;

    ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
    ros::Publisher control_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    Model::State rosbot_main_goal = {1. ,0. ,0.};
    const std::vector<float> q = runPSO(rosbot_main_goal);

    // main loop
    size_t i = 0;
    float time = 0.;
    float initial_start_time = ros::Time::now().toSec();
    float start_time = ros::Time::now().toSec();
    ros::Rate rate(1. / dt);
    
    // set first target point
    rosbot_goal = {q[i], q[i+1], q[i+2]};
    rosbot_goal.print();
    nop_controller.setGoal(rosbot_goal);
    
    // control msg
    geometry_msgs::Twist ctrl;
    while (ros::ok()) {
        ros::spinOnce();
        if (time >= TIME_STEP && !(ros::Time::now().toSec() - initial_start_time > MAX_TIME))
        {
            time = 0.;
            i = i + 3;
            rosbot_goal = {q[i], q[i+1], q[i+2]};
            rosbot_goal.print();
            nop_controller.setGoal(rosbot_goal);
        }

        if (rosbot_state.distXY(rosbot_main_goal) < EPS_MED) 
        {
            ctrl.linear.x = 0.;
            ctrl.angular.z = 0.;
            control_pub.publish(ctrl);
            time = time + ros::Time::now().toSec() - start_time; // time spend
            start_time = ros::Time::now().toSec();
            rate.sleep();  
            continue;
        }
        const Model::Control& u = nop_controller.calcControl(rosbot_state);
        ctrl.linear.x = u.left;
        ctrl.angular.z = u.right;

        
        control_pub.publish(ctrl);

        time = time + ros::Time::now().toSec() - start_time; // time spend
        start_time = ros::Time::now().toSec();

        rate.sleep();

    }
    control_pub.publish(geometry_msgs::Twist{});
    return 0;



}
