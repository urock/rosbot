#pragma once
#include <iostream>
#include <vector>
#include <cmath>

class Model {

public:
  struct Control {
    float left;
    float right;

    Control operator+(const Control &ctrl) const {
      return Control{this->left + ctrl.left, this->right + ctrl.right};
    }
    Control operator-(const Control &ctrl) const {
      return Control{this->left - ctrl.left, this->right - ctrl.right};
    }
    Control operator*(float val) const {
      return Control{this->left * val, this->right * val};
    }

  };

  struct State {
    float x;
    float y;
    float yaw;

    State operator+(const State &state) const {
      return State{this->x + state.x, this->y + state.y, this->yaw + state.yaw};
    }
    State operator-(const State &state) const {
      const float &yaw1=this->yaw;
      const float &yaw2=state.yaw;
      float dyaw = atan2f(sinf(yaw1 - yaw2), cosf(yaw1 - yaw2));
      return State{this->x - state.x, this->y - state.y, dyaw};
      // return State{this->x - state.x, this->y - state.y, this->yaw - state.yaw};

    }
    State operator*(float val) const {
      return State{this->x * val, this->y * val, this->yaw * val};
    }

    bool operator==(const State &state) const {
      return (this->x == state.x)&& (this->y == state.y)&& (this->yaw == state.yaw);
    }

    float dist(const State &state) const {
        auto ds = this->operator-(state);
        // return std::sqrt(ds.x * ds.x + ds.y * ds.y + ds.yaw * ds.yaw);
        return std::sqrt(ds.x * ds.x + ds.y * ds.y);
    }

  };

  Model(const State &, float);

  void setState(const State &);
  void setState(const Control &);
  const State &getState();

  State calcDeltaState(const Control &);

  State calcState(const Control &);
  State calcState(const State &);

private:
  float k = 0.5f;

  State mCurrentState;
  float dt;
};
