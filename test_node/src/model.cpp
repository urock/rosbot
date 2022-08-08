#include "test_node/model.hpp"
#include <cmath>

Model::State operator*(float val, const Model::State st) { return st * val; }

Model::Model(const Model::State &state_, float dt_)
    : mCurrentState(state_), dt(dt_) {}

void Model::setState(const State &state_) { mCurrentState = state_; }
void Model::setState(const Control &u) {
  mCurrentState = mCurrentState + calcDeltaState(u);
}

const Model::State &Model::getState() { return mCurrentState; }

Model::State Model::calcDeltaState(const Control &u) {
  return dt * k *
         State{(u.left + u.right) * cosf(mCurrentState.yaw),
               (u.left + u.right) * sinf(mCurrentState.yaw),
               (u.left - u.right)};
}

Model::State Model::calcState(const State &Vs) { return mCurrentState + Vs; }

Model::State Model::calcState(const Control &u) {
  return calcState(calcDeltaState(u));
}
