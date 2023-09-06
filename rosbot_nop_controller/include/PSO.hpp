#pragma once

#include <iostream>
#include <vector>
#include <limits>
#include <float.h>
#include <random>

#include "nop.hpp"
#include "model.hpp"
#include "controller.hpp"
#include "runner.hpp"

constexpr float EPS = 0.05;
constexpr float EPS_MED = 0.15; 

void run_to_goal(Model::State& currState, const Model::State& Goal, const float dt, const float max_time);

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
    Model::State main_goal;
    Particle() = default;
    Particle(const std::vector<float>& initial_state, Model::State main_goal, const float time_step, const float dt);

    void update_state();
    void update_velocities(std::vector<float> best_global_state);
    void evaluate();

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
    Model::State main_goal;
    
    PSO(std::vector<float> initial_state, size_t numParticles, size_t maxIter, Model::State main_goal, float time_step, float dt);
    std::vector<float> fit();

};
