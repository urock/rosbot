#include "rosbot_model.hpp"


Model::Model(const Model::State& state, float dt): 
  m_currentState(state), 
  dt(dt) 
  {}

void Model::setState(const State &newState)
{
  m_currentState = newState; 
}

void Model::setState(const Control &u) 
{
  m_currentState = m_currentState + calcDeltaState(u);
}

const Model::State &Model::getState()
{
  return m_currentState;
}

Model::State operator*(float val, const Model::State st) 
{ 
  return st * val; 
}

Model::State Model::calcDeltaState(const Control &u) 
{
  return dt * k *
         State{(u.left + u.right) * cosf(m_currentState.yaw),
               (u.left + u.right) * sinf(m_currentState.yaw),
               (u.left - u.right)};
}

Model::State Model::calcState(const State &Vs){
  return m_currentState + Vs; 
}

Model::State Model::calcState(const Control &u)
{
  return calcState(calcDeltaState(u));
}