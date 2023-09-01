#include <iostream>
#include <vector>
#include <limits>
#include <float.h>
#include <random>

#include "nop.hpp"
#include "model.hpp"
#include "controller.hpp"
#include "runner.hpp"



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
    Model::State MainGoal = {1.,1.,1.};
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

int main()
{

    // V COST FUNCTION NUZHNO PEREDAVAT DRUGOI MAX TIME, KOTORII delta_t tipa vremya vezdu tochkami
	std::cout<<"PSO START"<<std::endl;
    float Tmax = 6; // sec, 
    float time_step = 2; // sec
    float dt = 0.01; // sec
    std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)

    size_t numParticles = 100;
    size_t maxIter = 50;
    auto pso = PSO(q, numParticles, maxIter, Tmax, time_step, dt);

    
    // std::cout<< N <<std::endl;
    std::cout<<"q: ";
    for(auto i : q)
        std::cout<<i<<" ";
    std::cout<<std::endl<<"Result: ";
    for(auto i : pso.best_global_state)
        std::cout<<i<<" ";
    std::cout<<std::endl;



    Model::State currState = {0., 0., 0.};
    // Model::State MainGoal = {1.,1.,1.};
    std::cout<<"SIZE: "<<pso.best_global_state.size()<<std::endl;
    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
        run_to_goal(currState, Goal, dt, time_step);
    }
    std::cout<< "RESULT POSITION: " <<std::endl;
    std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<" "<<std::endl;

	return 0;
}