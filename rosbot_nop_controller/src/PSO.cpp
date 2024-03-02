#include "PSO.hpp"


// обрезать точки незя
// в каждый момент времени должен быть атрактор
// расстоения между точками от 1 до 2 не больше 1,5 метров
// график для модели, и точки куда едут которые найды в PSO
// график что едем из всех точек
// когда располагаем управляющие точки посмотреть где они
// 

namespace pso
{

void run_to_goal(Model::State& currState, const Model::State& Goal, const float dt, const float max_time, float& time_spent)
{
    // std::cout<<"run_to_goal dt"<<dt<<" max_time "<<max_time<<std::endl;
    NetOper nop = NetOper();
    nop.setLocalTestsParameters();

    Model model(currState, dt);
    Controller controller(Goal, nop);

    Runner runner(model, controller); 
    runner.init(currState);
    runner.setGoal(Goal);

    float time = 0.;                 
    while ((time < max_time) && (currState.dist(Goal) > EPS)) 
    {
        // std::cout<<"run_to_goal inside time"<<time<<" max_time "<<max_time<<"currState.dist(Goal)"<<currState.dist(Goal)<<std::endl;
        currState = runner.makeStep();
        time += dt;
    }
    time_spent = time_spent + time;
}


// TODO добавить время
float CostFunction(std::vector<float> points, Model::State MainGoal, const float step_time, const float dt)
{
    Model::State currState = {0., 0., 0.}; // начальная позиция
    float time_spent = 0.;
    for(size_t i = 0; i < points.size(); i = i + 3)
    {
        Model::State Goal = {points[i], points[i+1], points[i+2]};
        // Goal.print();
        run_to_goal(currState, Goal, dt, step_time, time_spent);
    }

    // Добавляем штраф за время
    float time_penalty_factor = 0.; // Этот коэффициент можно настроить
    float time_penalty = time_spent * time_penalty_factor;

    // Расчет общей стоимости с учетом штрафа за время
    return std::sqrt(currState.dist(MainGoal) * currState.dist(MainGoal)) + time_penalty; 
}


Particle::Particle(const std::vector<float>& initial_state, Model::State main_goal, const float step_time, const float dt):
main_goal(main_goal), step_time(step_time), dt(dt)
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

void Particle::evaluate()
{
    curr_error = CostFunction(curr_state, main_goal, step_time, dt);
    if (curr_error <= best_error)
    {
        best_state = curr_state;
        best_error = curr_error;
    }
    
}

void Particle::mutate(float mutation_rate, const std::vector<float>& min_bounds, const std::vector<float>& max_bounds) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        
        for (size_t i = 0; i < N; ++i) 
        {
            if (dis(gen) < mutation_rate) // Проверяем, произойдет ли мутация
            { 
                std::uniform_real_distribution<> dis_pos(min_bounds[i], max_bounds[i]);
                curr_state[i] = dis_pos(gen); // Присваиваем новое случайное значение из допустимого диапазона
            }
        }
    }

void Particle::update_velocities(std::vector<float> best_global_state, int iteration, int max_iterations)
{
    // Динамическая адаптация весов
    float w_end = 0.4;
    float w_start = 0.9;
    float w = w_end + (w_start - w_end) * (max_iterations - iteration) / max_iterations; // Линейно уменьшаем инерцию со временем

    float c1_start = 2.5, c1_end = 0.5;
    float c1 = c1_end + (c1_start - c1_end) * (max_iterations - iteration) / max_iterations;  // Уменьшаем когнитивный компонент

    float c2_start = 0.5, c2_end = 2.5;
    float c2 = c2_end + (c2_start - c2_end) * (max_iterations - iteration) / max_iterations; // Увеличиваем социальный компонент


    // Ограничения скорости
    float max_velocity = 0.1; // Максимальная скорость
    float min_velocity = -0.1; // Минимальная скорость

    for(size_t i = 0; i < N; ++i){
        float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float r2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // Вычисление новой скорости
        float vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
        float vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
        velocities[i] = w * velocities[i] + vel_cognitive + vel_social;

        // Применение ограничений скорости
        velocities[i] = std::max(min_velocity, std::min(max_velocity, velocities[i]));
    }
}

void Particle::update_state()
{
    for (size_t i = 0; i < N; ++i)
    {
        // std::cout<<"state was "<<curr_state[i];
        
        curr_state[i]=curr_state[i] + velocities[i];
        
        if (curr_state[i] < -3.0)
        {
            curr_state[i] = -3.0;
        }
        
        if (curr_state[i] > 3.0 )
        {
            curr_state[i] = 3.0;
        }
        
        //std::cout<<" state became "<<curr_state[i]<<std::endl;
    }
}


PSO::PSO(std::vector<float> initial_state, size_t numParticles, size_t maxIter, Model::State main_goal, float step_time, float dt): 
    maxIter(maxIter)
    , dt(dt)
    , step_time(step_time)
    , main_goal(main_goal)
{   
    best_global_state = initial_state;
    swarm.resize(numParticles);
    for(size_t i = 0; i < swarm.size(); ++i)
        swarm[i] = Particle(initial_state, main_goal, step_time, dt);

    fit();
}


std::vector<float> PSO::fit()
{
    for(size_t i = 0; i < maxIter; ++i)
    {  
        for (auto& p : swarm)
        {
            p.mutate(0.7, {-3.0, -3.0, -3.0}, {3.0, 3.0, 3.0});
            p.evaluate();
            //std::cout<<"!!! best global state update p.best_error " << p.best_error << " best_global_error " <<best_global_error<<std::endl;
            if (p.curr_error <= best_global_error)
            {
                //std::cout<<"!!! best global state update p.best_error " << p.best_error << " best_global_error " <<best_global_error<<std::endl;
                best_global_state=p.curr_state;
                best_global_error=p.curr_error;
            }

            p.update_velocities(best_global_state, i, maxIter);
            p.update_state();
        }
    }

    for (auto& p : swarm)
    {
        // std::cout<<"FINAL"<<std::endl;
        p.evaluate();

        // std::cout<<"!!! best global state update p.best_error " << p.best_error << " best_global_error " <<best_global_error<<std::endl;
        if (p.curr_error <= best_global_error)
        {
            // std::cout<<"!!! best global state update p.best_error " << p.best_error << " best_global_error " <<best_global_error<<std::endl;
            best_global_state=p.curr_state;
            best_global_error=p.curr_error;
        }
    }

    return best_global_state;
}

};

using namespace pso;

// int main()
// {
// 	std::cout<<"PSO START"<<std::endl;
//     float step_time = 1; // sec
//     float dt = 0.01; // sec
//     size_t numParticles = 100;
//     size_t maxIter = 200;
//     Model::State main_goal = {1.,1.,1.};
//     std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized
    
//     auto pso = PSO(q, numParticles, maxIter, main_goal, step_time, dt);

//     std::cout<<"q: ";
//     for(auto i : q)
//         std::cout<<i<<" ";
//     std::cout<<std::endl<<"Result: ";
//     for(auto i : pso.best_global_state)
//         std::cout<<i<<" ";
//     std::cout<<std::endl;


//     Model::State currState = {0., 0., 0.};
//     float time_spent;
 
//     for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
//     {
//         Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
//         std::cout<<"Goal "<<std::endl;
//         Goal.print();
//         run_to_goal(currState, Goal, dt, step_time, time_spent);
//     }
//     std::cout<< "RESULT POSITION and time: " <<std::endl;
//     std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<" "<<time_spent<<std::endl;

// 	return 0;
// }