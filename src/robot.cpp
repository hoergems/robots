#include "include/robot.hpp"

using std::cout;
using std::endl;

namespace shared {

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),
	constraints_enforced_(true),
	propagator_(nullptr),
	viewer_(nullptr),
	state_covariance_matrix_(),
	observation_covariance_matrix_(),
	goal_position_(),
	goal_radius_(){
	
}

bool Robot::propagateState(std::vector<double> &current_state,
	                       std::vector<double> &control_input,
	                       std::vector<double> &control_error,
	                       double duration,
	                       double simulation_step_size,	                       
	                       std::vector<double> &result) {
	/**cout << "current state: ";
	for (auto &k: current_state) {
		cout << k << ", " << endl;
	}
	cout << endl;
	cout << "control input: ";
	for (auto &k: control_input) {
		cout << k << ", " << endl;
	}
	cout << endl;
	
	cout << "control error: ";
	for (auto &k: control_error) {
		cout << k << ", " << endl;
	}
	cout << endl;
	
	cout << "duration: " << duration << endl;
	cout << "simulation_step_size: " << simulation_step_size << endl;*/
 	propagator_->propagateState(current_state,
			                    control_input,
			                    control_error,
			                    duration,
			                    simulation_step_size,			
			                    result);
 	/**cout << "result: ";
 	for (auto &k: result) {
 		cout << k << ", " << endl;
 	}
 	cout << endl;*/
	if (constraints_enforced_) {		
		return enforceConstraints(result);
	}
	
	return true;
}

void Robot::setGoalArea(std::vector<double> &goal_position, double &goal_radius) {
	goal_position_.clear();
	for (size_t i = 0; i < goal_position.size(); i++) {
		goal_position_.push_back(goal_position[i]);
	}
	
	goal_radius_ = goal_radius;
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
	assert(state_covariance_matrix_.rows() != 0 && "Robot: ERROR: State covariance matrix has not been set.");
	state_covariance_matrix = state_covariance_matrix_;
}

void Robot::setObservationCovarianceMatrix(Eigen::MatrixXd &observation_covariance_matrix) {	
	observation_covariance_matrix_ = observation_covariance_matrix;
}

void Robot::getObservationCovarianceMatrix(Eigen::MatrixXd &observation_covariance_matrix) const{
	assert(observation_covariance_matrix_.rows() != 0 && "Robot: ERROR: Observation covariance matrix has not been set.");
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