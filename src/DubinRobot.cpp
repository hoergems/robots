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

int DubinRobot::getStateSpaceDimension() {
	return 4;
}

int DubinRobot::getControlSpaceDimension() {
	return 2;
}

int getDOF() {
	return 4;
}

void DubinRobot::getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	
}

void DubinRobot::getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	
}

}