#pragma once

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "model.hpp"
#include "nop.hpp"


// class network operator
class Controller {

public:
  Controller(const Model::State &goalState, NetOper &netOper);

  /// RP from pascal version
  Model::Control calcControl(const Model::State &currState);

  // /// NormdistBetweenStateAndGoal
  // float distToGoal(const Model::State &currState);
  /// set new goal state
  void setGoal(Model::State newGoal);

  /// returns nnetwork operator
  NetOper &netOper();

private:
  Model::State mGoal;
  NetOper m_netOper;
  const float Eps = 0.1; // [m]
};
