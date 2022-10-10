#include "rosbot_controller.hpp"

Controller::Controller(const Model::State &goal_, const Model::State& m_initialState_, NetOper &netOper):mGoal(goal_)
	,m_netOper(netOper), m_initialState(m_initialState_)
{}

Model::Control Controller::calcControl(const Model::State& currState)
{
	Model::State delta; 


	delta.x = sqrtf((mGoal.x - currState.x)*(mGoal.x - currState.x) + (mGoal.y - currState.y)*(mGoal.y - currState.y));
	delta.y = (currState.x - m_initialState.x) * (mGoal.y - m_initialState.y) - (currState.y - m_initialState.y) * (mGoal.x - m_initialState.x);;

	std::cout<< delta.y<< std::endl;

	delta.yaw = mGoal.yaw - currState.yaw;

	delta.yaw = (delta.yaw < -M_PI)? delta.yaw + 2 * M_PI : delta.yaw;
	delta.yaw = (delta.yaw >  M_PI)? delta.yaw - 2 * M_PI : delta.yaw; 

	std::vector<float> ctrl(2,0);

	m_netOper.calcResult({delta.x, delta.y, delta.yaw}, ctrl);

	ctrl[0] = std::min(std::max(ctrl[0], -10.0f), 10.0f);
	ctrl[1] = std::min(std::max(ctrl[1], -10.0f), 10.0f);

	return {ctrl[0], ctrl[1]};
}

void Controller::setGoal(Model::State newGoal, const Model::State& initialState)
{
	if(mGoal == newGoal) return;
	mGoal = newGoal;
	m_initialState = initialState;
}

NetOper& Controller::netOper()
{
	return m_netOper;
}