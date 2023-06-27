#include <iostream>
#include <vector>
#include <limits>
#include <float.h>
#include <random>

#include "nop.hpp"
#include "model.hpp"
#include "controller.hpp"
#include "runner.hpp"



void run_to_goal(Model::State& currState, const Model::State& Goal)
{
    std::string output;
    bool stopInGoal = true;
    float maxTime = 0.5;
    std::string pathToMatrix = "/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461";
    std::string pathToParams = "/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt";

    //////////////
    constexpr float dt = 0.1;      
    constexpr float eps = 0.05; 

    //std::cout<<"START\n";
    //std::cout<<"Initial state = "<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<"\n";
    //std::cout<<"Goal state = "<<Goal.x<<" "<<Goal.y<<" "<<Goal.yaw<<"\n";
    
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
    runner.setGoal(Goal);

    double time = 0.;                 

    while (maxTime > time) 
    {
        currState = runner.makeStep();
        // currState.print();

        time += dt;
        if (stopInGoal && currState.dist(Goal) < eps)
            break; 
    }
    // std::cout<<"State ";
    // currState.print();
    //std::cout<<"spend time: " << time <<" (s)\n";
    ///std::cout<<"END\n";
}


double CostFunction(std::vector<double> points)
{
    Model::State currState = {0., 0., 0.};
    Model::State MainGoal = {1.,1.,1.};
    for(size_t i = 0; i < points.size(); i = i + 3)
    {
        Model::State Goal = {points[i], points[i+1], points[i+2]};
        run_to_goal(currState, Goal);
    }

    // double cost = 0.;
    // for (auto p : points)
    //     cost = cost + p*p;
    // std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<std::endl;
    return currState.dist(MainGoal) * currState.dist(MainGoal) + 5; 
}

class Particle
{
public:

    std::vector<double> curr_state;
    std::vector<double> best_state;
    std::vector<double> velocities; 
    double curr_error = std::numeric_limits<double>::max();
    double best_error = std::numeric_limits<double>::max();
    size_t N = 0; // number of state dimenstions

    Particle() = default;

    Particle(const std::vector<double>& initial_state)
    {
        // set random velocities
        N = initial_state.size();
        curr_state = initial_state;
        best_state = initial_state;

        velocities.resize(N);

        for(auto& v : velocities)
        {
            v = (double)rand() / ((double)RAND_MAX + 1);
        }
    }

    void evaluate()
    {
        curr_error = CostFunction(curr_state);
        if (curr_error <= best_error)
        {
            best_state = curr_state;
            best_error = curr_error;
        }
        
    }

    void update_velocities(std::vector<double> best_global_state)
    {
        double w=0.5;   // constant inertia weight (how much to weigh the previous velocity)
        double c1=1;    // cognative constant
        double c2=2;    // social constant
        

        for(size_t i = 0; i < N; ++i){
            double r1 = (double)rand() / ((double)RAND_MAX + 1);
            double r2 = (double)rand() / ((double)RAND_MAX + 1);
            double vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
            double vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
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
    std::vector<double> best_global_state;
    std::vector<Particle> swarm;
    size_t maxIter;
    double best_global_error = std::numeric_limits<double>::max();
    double time_step;

    PSO(std::vector<double> initial_state, size_t numParticles, size_t maxIter, double time_step): 
        maxIter(maxIter)
        ,time_step(time_step)
    {   
        best_global_state = initial_state;
        // init particles swarm
        swarm.resize(numParticles);
        for(size_t i = 0; i < swarm.size(); ++i)
            swarm[i] = Particle(initial_state);

        fit();
    }

    std::vector<double> fit()
    {
        for(size_t i = 0; i < maxIter; ++i)
        {  
            std::cout<<i<<" "<<best_global_error<<std::endl;
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

int main()
{
    // try diff delta_t 
    // 
	std::cout<<"PSO START"<<std::endl;
    double Tmax = 5; // sec, 
    double delta_t = 0.5; // sec
    double time_step = 0.1; // sec
    size_t N = static_cast<size_t>(Tmax / delta_t); // number of points to be optimized
    std::vector<double> q = {0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)

    size_t numParticles = 200;
    size_t maxIter = 50;
    auto pso = PSO(q, numParticles, maxIter, time_step);

    
    // std::cout<< N <<std::endl;
    std::cout<<"q: ";
    for(auto i : q)
        std::cout<<i<<" ";
    std::cout<<std::endl<<"Result: ";
    for(auto i : pso.best_global_state)
        std::cout<<i<<" ";
    std::cout<<std::endl;



    Model::State currState = {0., 0., 0.};
    Model::State MainGoal = {1.,1.,1.};
    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
        run_to_goal(currState, Goal);
    }
    std::cout<< "RESULT POSITION: " <<std::endl;
    std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<" "<<std::endl;

	return 0;
}