#include <iostream>
#include <vector>
#include <limits>
#include <float.h>
#include <random>

#include "nop.hpp"
#include "model.hpp"
#include "controller.hpp"
#include "runner.hpp"


// void run_to_goal(Model::State& currState, const Model::State& Goal)
// {
//     std::string output;
//     bool stopInGoal = true;
//     float maxTime = 0.5; // delta_t
//     std::string pathToMatrix = "/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461";
//     std::string pathToParams = "/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt";

//     //////////////
//     constexpr float dt = 0.1;      
//     constexpr float eps = 0.05; 
    
//     // create network operator and set prarams
//     NetOper nop = NetOper();
//     nop.setNodesForVars({0, 1, 2});      // Pnum
//     nop.setNodesForParams({3, 4, 5});    // Rnum
//     nop.setNodesForOutput({22, 23});     // Dnum
//     nop.setCs(qc);                       // set Cs
//     nop.setPsi(NopPsiN);                 // set matrix

//     // u can read params with reader
//     if(pathToMatrix.size() > 0)
//     {
//         // std::cout<<"Matrix path "<<pathToMatrix<<std::endl;
//         NOPMatrixReader& reader = nop.getReader();
//         reader.readMatrix(pathToMatrix);
//         nop.setPsi(reader.getMatrix());
//     }
//     if(pathToParams.size() > 0)
//     {
//         // std::cout<<"Matrix parameters "<<pathToParams<<std::endl;
//         NOPMatrixReader& reader = nop.getReader();
//         reader.readParams(pathToParams);
//         nop.setCs(reader.getParams());
//     }

//     Model model(currState, dt);

//     Controller controller(Goal, nop);

//     Runner runner(model, controller); 
//     runner.setGoal(Goal);

//     double time = 0.;                 

//     while (maxTime > time) 
//     {
//         currState = runner.makeStep();
//         // currState.print();

//         time += dt;
//         if (stopInGoal && currState.dist(Goal) < eps)
//             break; 
//     }
//     // std::cout<<"State ";
//     // currState.print();
//     //std::cout<<"spend time: " << time <<" (s)\n";
//     ///std::cout<<"END\n";
// }


// double CostFunction(std::vector<double> points)
// {
//     Model::State currState = {0., 0., 0.};
//     // Model::State MainGoal = {-1.,-2.,-1.};
//     for(size_t i = 0; i < points.size(); i = i + 3)
//     {
//         Model::State Goal = {points[i], points[i+1], points[i+2]};
//         run_to_goal(currState, Goal);
//     }

//     // double cost = 0.;
//     // for (auto p : points)
//     //     cost = cost + p*p;
//     // std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<std::endl;
//     return currState.dist(rosbot_main_goal) * currState.dist(rosbot_main_goal); 
// }

// class Particle
// {
// public:

//     std::vector<double> curr_state;
//     std::vector<double> best_state;
//     std::vector<double> velocities; 
//     double curr_error = std::numeric_limits<double>::max();
//     double best_error = std::numeric_limits<double>::max();
//     size_t N = 0; // number of state dimenstions

//     Particle() = default;

//     Particle(const std::vector<double>& initial_state)
//     {
//         // set random velocities
//         N = initial_state.size();
//         curr_state = initial_state;
//         best_state = initial_state;

//         velocities.resize(N);

//         for(auto& v : velocities)
//         {
//             v = (double)rand() / ((double)RAND_MAX + 1);
//         }
//     }

//     void evaluate()
//     {
//         curr_error = CostFunction(curr_state);
//         if (curr_error <= best_error)
//         {
//             best_state = curr_state;
//             best_error = curr_error;
//         }
        
//     }

//     void update_velocities(std::vector<double> best_global_state)
//     {
//         double w=1.5;   // constant inertia weight (how much to weigh the previous velocity)
//         double c1=1;    // cognative constant
//         double c2=10;    // social constant
        

//         for(size_t i = 0; i < N; ++i){
//             double r1 = (double)rand() / ((double)RAND_MAX + 1);
//             double r2 = (double)rand() / ((double)RAND_MAX + 1);
//             double vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
//             double vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
//             velocities[i] = w * velocities[i] + vel_cognitive + vel_social;
//         }
//     }

//     void update_state()
//     {
//         for (size_t i = 0; i < N; ++i)
//         {
//             curr_state[i]=curr_state[i] + velocities[i];
//         }
//     }
// };

// class PSO
// {
// public:
//     std::vector<double> best_global_state;
//     std::vector<Particle> swarm;
//     size_t maxIter;
//     double best_global_error = std::numeric_limits<double>::max();
//     double time_step;

//     PSO(std::vector<double> initial_state, size_t numParticles, size_t maxIter, double time_step): 
//         maxIter(maxIter)
//         ,time_step(time_step)
//     {   
//         best_global_state = initial_state;
//         // init particles swarm
//         swarm.resize(numParticles);
//         for(size_t i = 0; i < swarm.size(); ++i)
//             swarm[i] = Particle(initial_state);

//         fit();
//     }

//     std::vector<double> fit()
//     {
//         for(size_t i = 0; i < maxIter; ++i)
//         {  
//             std::cout<<i<<" "<<best_global_error<<std::endl;
//             for (auto& p : swarm)
//             {
//                 p.evaluate();
//                 if (p.curr_error < best_global_error)
//                 {
//                     best_global_state=p.curr_state;
//                     best_global_error=p.curr_error;
//                 }

//                 p.update_velocities(best_global_state);
//                 p.update_state();
//             }
//         }
//         return best_global_state;
//     }

// };

void run_to_goal(Model::State& currState, const Model::State& Goal, const float dt, const float time_step)
{
    bool stopInGoal = true;
    std::string pathToMatrix = "/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461";
    std::string pathToParams = "/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt";

    //////////////     
    constexpr float eps = 0.05; 

    //std::cout<<"START\n";
    //std::cout<<"Initial state = "<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<"\n";
    // std::cout<<"Goal state = "<<Goal.x<<" "<<Goal.y<<" "<<Goal.yaw<<"\n";
    
    // create network operator and set prarams
    NetOper nop = NetOper();
    nop.setNodesForVars({0, 1, 2});      // Pnum
    nop.setNodesForParams({3, 4, 5});    // Rnum
    nop.setNodesForOutput({22, 23});     // Dnum
    nop.setCs(qc);                       // set Cs
    nop.setPsi(NopPsiN);                 // set matrix

    // u can read params with reader
    if(pathToMatrix.size() > 0)
    {
        // std::cout<<"Matrix path "<<pathToMatrix<<std::endl;
        NOPMatrixReader& reader = nop.getReader();
        reader.readMatrix(pathToMatrix);
        nop.setPsi(reader.getMatrix());
    }
    if(pathToParams.size() > 0)
    {
        // std::cout<<"Matrix parameters "<<pathToParams<<std::endl;
        NOPMatrixReader& reader = nop.getReader();
        reader.readParams(pathToParams);
        nop.setCs(reader.getParams());
    }

    Model model(currState, dt);

    Controller controller(Goal, nop);

    Runner runner(model, controller); 
    runner.init(currState);
    runner.setGoal(Goal);

    float time = 0.;                 

    while (time < time_step) 
    {
        currState = runner.makeStep();
        // currState.print();

        time += dt;
        if (stopInGoal && currState.dist(Goal) < eps)
            break; 
    }
    // std::cout<<"State ";
    // currState.print();
    // std::cout<<"Goal state = "<<Goal.x<<" "<<Goal.y<<" "<<Goal.yaw<<"\n";
    // std::cout<<"spend time: " << time <<" (s)\n";
    // std::cout<<"END\n";
}


float CostFunction(std::vector<float> points, const float Tmax, const float time_step, const float dt)
{
    Model::State currState = {0., 0., 0.};
    Model::State MainGoal = {1.,0.,0.};
    for(size_t i = 0; i < points.size(); i = i + 3)
    {
        Model::State Goal = {points[i], points[i+1], points[i+2]};
        run_to_goal(currState, Goal, dt, time_step);
    }
    
    return std::sqrt(currState.dist(MainGoal) * currState.dist(MainGoal)); 
}

class Particle
{
public:

    std::vector<float> curr_state;
    std::vector<float> best_state;
    std::vector<float> velocities; 
    float curr_error = std::numeric_limits<float>::max();
    float best_error = std::numeric_limits<float>::max();
    size_t N = 0; // number of state dimenstions
    float Tmax = 0.;
    float time_step = 0.;
    float dt = 0.;
    Particle() = default;

    Particle(const std::vector<float>& initial_state, const float Tmax, const float time_step, const float dt):
    Tmax(Tmax), time_step(time_step), dt(dt)
    {
        // set random velocities
        N = initial_state.size();
        curr_state = initial_state;
        best_state = initial_state;
        velocities.resize(N);

        for(auto& v : velocities)
        {
            v = (float)rand() / ((float)RAND_MAX + 1);
        }
    }

    void evaluate()
    {
        curr_error = CostFunction(curr_state, Tmax, time_step, dt);
        if (curr_error <= best_error)
        {
            best_state = curr_state;
            best_error = curr_error;
        }
        
    }

    void update_velocities(std::vector<float> best_global_state)
    {
        float w=0.5;   // constant inertia weight (how much to weigh the previous velocity)
        float c1=1;    // cognative constant
        float c2=2;    // social constant
        

        for(size_t i = 0; i < N; ++i){
            float r1 = (float)rand() / ((float)RAND_MAX + 1);
            float r2 = (float)rand() / ((float)RAND_MAX + 1);
            float vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
            float vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
            velocities[i] = w * velocities[i] + vel_cognitive + vel_social;
        }
    }

    void update_state()
    {
        for (size_t i = 0; i < N; ++i)
        {
            curr_state[i]=curr_state[i] + velocities[i];
        }
    }
};

class PSO
{
public:
    std::vector<float> best_global_state;
    std::vector<Particle> swarm;
    size_t maxIter;
    float best_global_error = std::numeric_limits<float>::max();
    float dt = 0.;
    float Tmax = 0.;
    float time_step = 0.;

    PSO(std::vector<float> initial_state, size_t numParticles, size_t maxIter, float Tmax, float time_step, float dt): 
        maxIter(maxIter)
        , dt(dt)
        , time_step(time_step)
        , Tmax(Tmax)
    {   
        best_global_state = initial_state;
        // init particles swarm
        swarm.resize(numParticles);
        for(size_t i = 0; i < swarm.size(); ++i)
            swarm[i] = Particle(initial_state, Tmax, time_step, dt);

        fit();
    }

    std::vector<float> fit()
    {
        for(size_t i = 0; i < maxIter; ++i)
        {  
            // std::cout<<i<<" "<<best_global_error<<std::endl;
            for (auto& p : swarm)
            {
                p.evaluate();
                if (p.curr_error < best_global_error)
                {
                    best_global_state=p.curr_state;
                    best_global_error=p.curr_error;
                }

                p.update_velocities(best_global_state);
                p.update_state();
            }
        }
        return best_global_state;
    }

};


std::vector<float> runPSO()
{

	std::cout<<"PSO START"<<std::endl;
    float Tmax = 6; // sec, 
    float time_step = 2; // sec
    float dt = 0.01; // sec
    std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)

    size_t numParticles = 100;
    size_t maxIter = 50;
    auto pso = PSO(q, numParticles, maxIter, Tmax, time_step, dt);

    std::cout<<"q: ";
    for(auto i : q)
        std::cout<<i<<" ";
    std::cout<<std::endl<<"Result: ";
    for(auto i : pso.best_global_state)
        std::cout<<i<<" ";
    std::cout<<std::endl;



    Model::State currState = {0., 0., 0.};
    std::cout<<"SIZE: "<<pso.best_global_state.size()<<std::endl;
    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
        run_to_goal(currState, Goal, dt, time_step);
    }
    std::cout<< "RESULT POSITION: " <<std::endl;
    std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<" "<<std::endl;

	return pso.best_global_state;

}

////// PSO OPTIMIZATION


#include "rosbot_controller.hpp"

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <cmath>
#include <iostream>

constexpr size_t rosbot_model_id = 1;
constexpr float dt = 0.01;
constexpr float epsterm = 0.1;

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

  NetOper netOp;

  NOPMatrixReader& reader = netOp.getReader();
  reader.readMatrix("/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461");
  reader.readParams("/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt");

  netOp.setNodesForVars({0, 1, 2});   // Pnum
  netOp.setNodesForParams({3, 4, 5}); // Rnum
  netOp.setNodesForOutput({22, 23});  // Dnum
  netOp.setCs(reader.getParams());
  netOp.setPsi(reader.getMatrix());

  RosbotNOPController nop_controller(rosbot_goal, netOp);

  ros::init(argc, argv, "rosbot_nop_controller");
  ros::NodeHandle node;

  // subscribers
  ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
  // ros::Subscriber target_sub = node.subscribe("clicked_point", 5, target_sub_cb);
  // publishers
  ros::Publisher control_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 5);


  const std::vector<float> q = runPSO();

  // main loop
  size_t i = 0;
  float time = 0.;
  float initial_start_time = ros::Time::now().toSec();
  float start_time = ros::Time::now().toSec();
  std::cout<<q[i]<<" "<<q[i+1]<<" "<<q[i+2]<<std::endl;
  ros::Rate rate(1. / dt);
  while (ros::ok()) {
    ros::spinOnce();
    if (time >= 2)
    {
        time = 0.;
        i = i + 3;
        std::cout<<"update goal"<<std::endl;
        std::cout<<q[i]<<" "<<q[i+1]<<" "<<q[i+2]<<std::endl;
    }
    rosbot_goal = {q[i], q[i+1], q[i+2]};
    // update_target_yaw();
    nop_controller.setGoal(rosbot_goal);

    geometry_msgs::Twist ctrl;
    Model::State rosbot_main_goal = {1, 0, 0.};
    if (rosbot_state.distXY(rosbot_main_goal) > epsterm) 
    {
      const Model::Control& u = nop_controller.calcControl(rosbot_state);
      ctrl.linear.x = u.left;
      ctrl.angular.z = u.right;

    }
    control_pub.publish(ctrl);

    // ROS_INFO("State: %lf %lf %lf \n", rosbot_state.x, rosbot_state.y, rosbot_state.yaw);
    // ROS_INFO("Target: %lf %lf %lf\n", rosbot_goal.x, rosbot_goal.y, rosbot_goal.yaw);
    // ROS_INFO("Control [cmd_vel]: %lf %lf\n", ctrl.linear.x, ctrl.angular.z);
    time = time + ros::Time::now().toSec() - start_time; // time spend
    start_time = ros::Time::now().toSec();
    //std::cout<<"time: "<<time<<std::endl;
    //std::cout<<q[i]<<" "<<q[i+1]<<" "<<q[i+2]<<std::endl;

    if (ros::Time::now().toSec() - initial_start_time > 6)
    {return 0;}

    rate.sleep();

  }

  return 0;



}
