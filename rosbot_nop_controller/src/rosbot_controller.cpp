#include "rosbot_controller.hpp"

Controller::Controller(const NetOper& netOper, const Model::State& startingState, const Model::State& goal):
	m_netOper(netOper),
	m_initialState(startingState),
	m_goal(goal)
	{}

Model::Control Controller::calcControl(const Model::State& currState)
{
	Model::State delta; 
	delta.x = sqrtf((m_goal.x - currState.x)*(m_goal.x - currState.x) + (m_goal.y - currState.y)*(m_goal.y - currState.y));
	delta.y = (currState.x - m_initialState.x) * (m_goal.y - m_initialState.y) - (currState.y - m_initialState.y) * (m_goal.x - m_initialState.x);

	delta.yaw = m_goal.yaw - currState.yaw;

	delta.yaw = (delta.yaw < -M_PI)? delta.yaw + 2 * M_PI : delta.yaw;
	delta.yaw = (delta.yaw >  M_PI)? delta.yaw - 2 * M_PI : delta.yaw; 

	std::vector<float> ctrl(2,0);

	m_netOper.calcResult({delta.x, delta.y, delta.yaw}, ctrl);

	ctrl[0] = std::min(std::max(ctrl[0], -10.0f), 10.0f);
	ctrl[1] = std::min(std::max(ctrl[1], -10.0f), 10.0f);

	return {ctrl[0], ctrl[1]};
}

void Controller::setGoal(const Model::State& startingState, const Model::State& goal)
{
	if(m_goal == goal) return;
	m_initialState = startingState;
	m_goal = goal;
}

NetOper& Controller::netOper()
{
	return m_netOper;
}