#include "include/robot.hpp"

using std::cout;
using std::endl;

namespace shared
{

Robot::Robot(std::string robot_file):
    robot_file_(robot_file),
    constraints_enforced_(true),
    propagator_(nullptr),
    viewer_(nullptr),
    state_covariance_matrix_(),
    observation_covariance_matrix_(),
    goal_position_(),
    goal_radius_(),
    lowerStateLimits_(),
    upperStateLimits_(),
    lowerControlLimits_(),
    upperControlLimits_()
{
#ifdef USE_OPENRAVE
    viewer_ = std::make_shared<shared::ViewerInterface>();    
#endif
}

bool Robot::propagateState(const std::vector<double>& current_state,
                           std::vector<double>& control_input,
                           std::vector<double>& control_error,
                           double duration,
                           double simulation_step_size,
                           std::vector<double>& result)
{
    boost::this_thread::interruption_point();
    result.clear();
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

void Robot::setGoalArea(std::vector<double>& goal_position, double& goal_radius)
{
    goal_position_.clear();
    for (size_t i = 0; i < goal_position.size(); i++) {
        goal_position_.push_back(goal_position[i]);
    }

    goal_radius_ = goal_radius;
}

bool Robot::checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const
{
    return false;
}

bool Robot::checkSelfCollision(const std::vector<double>& state) const
{
    return false;
}

bool Robot::constraintsEnforced()
{
    return constraints_enforced_;
}

void Robot::enforceConstraints(bool enforce)
{
    constraints_enforced_ = enforce;
}

bool Robot::enforceConstraints(std::vector<double>& state) const
{
    bool enforced = false;
    for (size_t i = 0; i < state.size(); i++) {
        if (state[i] < lowerStateLimits_[i]) {
            state[i] = lowerStateLimits_[i];
            enforced = true;
        }

        else if (state[i] > upperStateLimits_[i]) {
            state[i] = upperStateLimits_[i];
            enforced = true;
        }
    }

    return enforced;
}

bool Robot::enforceControlConstraints(std::vector<double>& control) const
{
    for (size_t i = 0; i < control.size(); i++) {
        if (control[i] < lowerControlLimits_[i]) {
            control[i] = lowerControlLimits_[i];
        }

        else if (control[i] > upperControlLimits_[i]) {
            control[i] = upperControlLimits_[i];
        }
    }
}

void Robot::sampleRandomControl(std::vector<double> &control, std::default_random_engine* randGen) {    
    control = std::vector<double>(lowerControlLimits_.size());    
    for (size_t i = 0; i < lowerControlLimits_.size(); i++) {
	std::uniform_real_distribution<double> uniform_dist(lowerControlLimits_[i], upperControlLimits_[i]);
	double rand_num = uniform_dist(*randGen);
	control[i] = rand_num;
    }
}

void Robot::getStateLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const
{
    lowerLimits = lowerStateLimits_;
    upperLimits = upperStateLimits_;
}

void Robot::getControlLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const
{
    lowerLimits = lowerControlLimits_;
    upperLimits = upperControlLimits_;
}

void Robot::setStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix)
{
    state_covariance_matrix_ = state_covariance_matrix;
}

void Robot::getStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) const
{
    assert(state_covariance_matrix_.rows() != 0 && "Robot: ERROR: State covariance matrix has not been set.");
    state_covariance_matrix = state_covariance_matrix_;
}

void Robot::setObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix)
{
    observation_covariance_matrix_ = observation_covariance_matrix;
}

void Robot::getObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) const
{
    assert(observation_covariance_matrix_.rows() != 0 && "Robot: ERROR: Observation covariance matrix has not been set.");
    observation_covariance_matrix = observation_covariance_matrix_;
}


void Robot::setupViewer(std::string model_file, std::string environment_file)
{
#ifdef USE_OPENRAVE
    viewer_->setupViewer(model_file, environment_file);
#endif
}


void Robot::setParticlePlotLimit(unsigned int particle_plot_limit)
{
#ifdef USE_OPENRAVE
    viewer_->setParticlePlotLimit(particle_plot_limit);
#endif
}

}
