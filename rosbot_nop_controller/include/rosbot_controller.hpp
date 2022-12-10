#pragma once

#include "controller.hpp"

class RosbotNOPController: public Controller
{

public:
  RosbotNOPController(const Model::State& goal, NetOper& netOper);

  Model::Control calcNOPControl(const Model::State& currState);

  Model::Control calcPropControl(const Model::State& currState);

  // void run();



private:
  Model::State m_prevState = Model::State();
};
