#include "include/DubinRobot.hpp"

namespace shared {

DubinRobot::DubinRobot(std::string robot_file):
	Robot(robot_file)
{
	propagator_ = std::make_shared<shared::DubinPropagator>();
	double d = 1.0;
	static_cast<shared::DubinPropagator *>(propagator_.get())->setD(d);
}

void DubinRobot::createRobotCollisionObjects(const std::vector<double> &state, 
		       std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const {
	
}

int DubinRobot::getStateSpaceDimension() const {
	return 4;
}

int DubinRobot::getControlSpaceDimension() const {
	return 2;
}

int DubinRobot::getDOF() const {
	return 4;
}

void DubinRobot::makeNextStateAfterCollision(std::vector<double> &previous_state,
				                             std::vector<double> &colliding_state,
				                             std::vector<double> &next_state) {
	next_state = previous_state;
}

bool DubinRobot::isTerminal(std::vector<double> &state) const {
	double dist = distanceGoal(state);
	if (dist < goal_radius_) {
		return true;
	}
	
	return false;
}

double DubinRobot::distanceGoal(std::vector<double> &state) const {
	assert(goal_position_.size() != 0 && "ManipulatorRobot: No goal area set. Cannot calculate distance!");
	double dist = 0.0;
	for (size_t i = 0; i < state.size(); i++) {
		dist += std::pow(state[i] - goal_position_[i], 2);
	}
		
	return std::sqrt(dist);
}

void DubinRobot::getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	
}

void DubinRobot::getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	
}

void DubinRobot::getLinearProcessMatrices(std::vector<double> &state, 
	    	    			              std::vector<double> &control, 
	    	    			              double &duration,
	    	    			              std::vector<Eigen::MatrixXd> &matrices) const {
	
}

}