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


std::vector<float> runPSO(Model::State main_goal, float max_time, float time_step)
{
    float dt = 0.01; // dt for control 
    size_t numParticles = 100;
    size_t maxIter = 50;
    size_t state_size = 3;
    size_t vector_size = static_cast<size_t>(max_time / time_step) * state_size;

    std::vector<float> q(vector_size, 0.); 
    auto pso_result = pso::PSO(q, numParticles, maxIter, main_goal, time_step, dt);

    Model::State currState{0., 0., 0.};
    float time_spent;
    for(size_t i = 0; i < pso_result.best_global_state.size(); i = i + 3)
    {
        Model::State Goal = {pso_result.best_global_state[i], pso_result.best_global_state[i+1], pso_result.best_global_state[i+2]};
        pso::run_to_goal(currState, Goal, dt, time_step, time_spent);
    }
	return pso_result.best_global_state;
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


// Добавить визуализацию RVIZ

int main(int argc, char **argv) 
{

    constexpr float  MAX_TIME = 3.; // T+
    constexpr float TIME_STEP = 1.;
    
    NetOper nop = NetOper();
    nop.setLocalTestsParameters();

    RosbotNOPController nop_controller(rosbot_goal, nop);

    ros::init(argc, argv, "rosbot_nop_controller");
    ros::NodeHandle node;
    ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
    ros::Publisher control_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    Model::State rosbot_main_goal = {2. ,1. ,0.};
    const std::vector<float> q = runPSO(rosbot_main_goal, MAX_TIME, TIME_STEP);

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
    float time_spent = 0.;
    while (ros::ok()) {
        ros::spinOnce();
        if ((time >= TIME_STEP || rosbot_state.distXY(rosbot_goal) < pso::EPS) && !(ros::Time::now().toSec() - initial_start_time > MAX_TIME))
        {
            time_spent += time;
            std::cout<<time_spent<<std::endl;
            time = 0.;
            i = i + 3;
            rosbot_goal = {q[i], q[i+1], q[i+2]};
            rosbot_goal.print();
            nop_controller.setGoal(rosbot_goal);
        }

        if (rosbot_state.distXY(rosbot_main_goal) < pso::EPS) 
        {
            ctrl.linear.x = 0.;
            ctrl.angular.z = 0.;
            control_pub.publish(ctrl);
            time = time + ros::Time::now().toSec() - start_time; // time spend
            start_time = ros::Time::now().toSec();
            rate.sleep();  
            break;
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
