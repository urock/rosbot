#pragma once

#include <cmath>
#include <vector>

#include "rosbot_model.hpp"
#include "nop.hpp"

// class network operator
class Controller {

public:
  Controller(const Model::State &goalState, const Model::State& m_initialState_, NetOper &netOper);

  /// RP from pascal version
  Model::Control calcControl(const Model::State &currState);

  // /// NormdistBetweenStateAndGoal
  // float distToGoal(const Model::State &currState);
  /// set new goal state
  void setGoal(Model::State newGoal, const Model::State& initialState);

  /// returns nnetwork operator
  NetOper &netOper();

private:
  Model::State mGoal;
  Model::State m_initialState;
  NetOper m_netOper;
  const float Eps = 0.1; // [m]
};
