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

bool DubinRobot::isTerminal(std::vector<double> &state) const {
	return false;
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

bool DubinRobot::enforceConstraints(std::vector<double> &state) const {
	return true;
}

}