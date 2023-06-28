#pragma once

#include "controller.hpp"

class RosbotNOPController: public Controller
{

public:

  enum Mode
  {
    NOP,
    Proportional
  };

  RosbotNOPController();
  
  RosbotNOPController(const Model::State& goal, NetOper& netOper);

  Model::Control calcControl(const Model::State& currState) override;

  void setMode(Mode newMode);

private:
  Model::Control calcNOPControl(const Model::State& currState);
  Model::Control calcPropControl(const Model::State& currState);

private:
  Model::State m_prevState = Model::State();

  float k = 0.025;
  float b = 0.21 * 2;
  Mode m_mode = Mode::NOP;
};
