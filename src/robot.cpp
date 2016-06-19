#include "include/robot.hpp"

using std::cout;
using std::endl;

namespace shared {

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),
	constraints_enforced_(false),
	propagator_(nullptr),
	viewer_(nullptr),
	state_covariance_matrix_(),
	observation_covariance_matrix_(){
	
}

bool Robot::propagateState(std::vector<double> &current_state,
	                       std::vector<double> &control_input,
	                       std::vector<double> &control_error,
	                       double duration,
	                       double simulation_step_size,	                       
	                       std::vector<double> &result) {
	propagator_->propagateState(current_state,
			                    control_input,
			                    control_error,
			                    duration,
			                    simulation_step_size,			
			                    result);
	if (constraints_enforced_) {
		return enforceConstraints(result);
	}
	
	return true;
}

bool Robot::checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const {
	return false;
}

bool Robot::checkSelfCollision(const std::vector<double> &state) const {
	return false;
}

bool Robot::constraintsEnforced() {
	return constraints_enforced_;	
}

void Robot::enforceConstraints(bool enforce) {
	constraints_enforced_ = enforce;
	propagator_->enforceConstraints(constraints_enforced_);
}

void Robot::setStateCovarianceMatrix(Eigen::MatrixXd &state_covariance_matrix) {
	state_covariance_matrix_ = state_covariance_matrix;
}

void Robot::getStateCovarianceMatrix(Eigen::MatrixXd &state_covariance_matrix) const{
	state_covariance_matrix = state_covariance_matrix_;
}

void Robot::setObservationCovarianceMatrix(Eigen::MatrixXd &observation_covariance_matrix) {
	observation_covariance_matrix_ = observation_covariance_matrix;
}

void Robot::getObservationCovarianceMatrix(Eigen::MatrixXd &observation_covariance_matrix) const{
	observation_covariance_matrix = observation_covariance_matrix_;
}

#ifdef USE_URDF
void Robot::setupViewer(std::string model_file, std::string environment_file) {	
	viewer_->setupViewer(model_file, environment_file);	
}

void Robot::setParticlePlotLimit(unsigned int particle_plot_limit) {	
	if (viewer_) {
		viewer_->setParticlePlotLimit(particle_plot_limit);
	}
}

#endif

}