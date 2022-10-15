#pragma once

#include "rosbot_model.hpp"
#include "nop.hpp"

#include <cmath>
#include <vector>

class Controller {

public:
  Controller(const NetOper& netOper, const Model::State& startingState, const Model::State& goal);

  /// RP from pascal version
  Model::Control calcNOPControl(const Model::State& currState);

  Model::Control calcPropControl(const Model::State& currState);

  void setGoal(const Model::State& startingState, const Model::State& goal);

  /// returns nnetwork operator
  NetOper& netOper();

private:
  Model::State m_goal;
  Model::State m_prevState;
  NetOper m_netOper;
};
