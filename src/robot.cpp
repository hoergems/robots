#include <robot_headers/robot.hpp>

using std::cout;
using std::endl;

namespace shared
{

Robot::Robot(std::string robot_file):
    InterfaceBase(),
    robot_file_(robot_file),
    constraints_enforced_(true),
    propagator_(nullptr),
    viewer_(nullptr),
    state_covariance_matrix_(),
    observation_covariance_matrix_(),
    goal_position_(),
    goal_radius_(),    
    process_distribution_(nullptr),
    observation_distribution_(nullptr),    
    environmentInfo_(nullptr),
    stateSpace_(nullptr),
    actionSpace_(nullptr),
    observationSpace_(nullptr)
{
#ifdef USE_OPENRAVE
    viewer_ = std::make_shared<shared::ViewerInterface>();
#endif
}

bool Robot::propagateState(const frapu::RobotStateSharedPtr& state,
                           const frapu::ActionSharedPtr& action,
                           const std::vector<double> controlError,
                           double duration,
                           double simulationStepSize,
                           frapu::RobotStateSharedPtr& result)
{
    boost::this_thread::interruption_point();
    result = nullptr;
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    std::vector<double> controlVec = static_cast<const frapu::VectorAction*>(action.get())->asVector();
    std::vector<double> resultVec;    
    propagator_->propagateState(stateVec, controlVec, controlError, duration, simulationStepSize, resultVec);
    result = std::make_shared<frapu::VectorState>(resultVec);
    if (constraints_enforced_) {
        stateSpace_->enforceStateLimits(result);        
    }    
    
    return true;
}

bool Robot::propagateState(const frapu::RobotStateSharedPtr& state,
                           const frapu::ActionSharedPtr& action,
                           double duration,
                           double simulationStepSize,
                           frapu::RobotStateSharedPtr& result)
{
    
    boost::this_thread::interruption_point();
    std::vector<double> controlVec = static_cast<const frapu::VectorAction*>(action.get())->asVector();
    std::vector<double> controlError(controlVec.size());
    Eigen::MatrixXd sample = process_distribution_->samples(1);
    for (size_t i = 0; i < controlError.size(); i++) {
        controlError[i] = sample(i, 0);
    }
    
    return propagateState(state, action, controlError, duration, simulationStepSize, result);    
}

bool Robot::isValid(const frapu::RobotStateSharedPtr& state) const
{
    std::vector<frapu::CollisionObjectSharedPtr> collisionObjects;
    createRobotCollisionObjects(state, collisionObjects);
    for (size_t i = 0; i < environmentInfo_->obstacles.size(); i++) {
        if (environmentInfo_->obstacles[i]->inCollision(collisionObjects)) {
            return false;
        }
    }

    return true;
}

void Robot::updateRobot(const frapu::RobotStateSharedPtr& state)
{

}

std::shared_ptr<Eigen::Distribution<double>> Robot::getProcessDistribution() const
{
    return process_distribution_;
}

std::shared_ptr<Eigen::Distribution<double>> Robot::getObservationDistribution() const
{
    return observation_distribution_;
}

double Robot::calcLikelihood(const frapu::RobotStateSharedPtr& state, std::vector<double>& observation)
{
    std::vector<double> transformedState;
    transformToObservationSpace(state, transformedState);
    return observation_distribution_->calcPdf(observation, transformedState);
}

frapu::StateSpaceSharedPtr Robot::getStateSpace() const {
    return stateSpace_;
}

frapu::ObservationSpaceSharedPtr Robot::getObservationSpace() const
{
    return observationSpace_;
}

frapu::ActionSpaceSharedPtr Robot::getActionSpace() const
{
    if (!actionSpace_) {
        assert(false && "ACTION SPACE IS NULL");
    }
    return actionSpace_;
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

bool Robot::checkSelfCollision(const frapu::RobotStateSharedPtr& state) const
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

unsigned int Robot::getControlSpaceDimension() const
{
    return actionSpace_->getNumDimensions();
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

void Robot::setNewtonModel()
{

}

void Robot::setGravityConstant(double gravity_constant)
{

}

void Robot::setEnvironmentInfo(frapu::EnvironmentInfoSharedPtr& environmentInfo)
{
    environmentInfo_ = environmentInfo;
}

void Robot::resetViewer(std::string model_file, std::string environment_file)
{
#ifdef USE_OPENRAVE
    viewer_->resetViewer(model_file, environment_file);
#endif
}

void Robot::setupViewer(std::string model_file, std::string environment_file)
{
#ifdef USE_OPENRAVE
    viewer_->setupViewer(model_file, environment_file);
#endif
}

void Robot::addBox(std::string name, std::vector<double> dims)
{
#ifdef USE_OPENRAVE
    viewer_->addObstacle(name, dims);
#endif
}

void Robot::removeBox(std::string name)
{
#ifdef USE_OPENRAVE
    viewer_->removeObstacle(name);
#endif
}

void Robot::getCameraImage(std::vector<uint8_t>& image, int width, int height)
{
#ifdef USE_OPENRAVE
    viewer_->getCameraImage(image, width, height);
#endif
}

void Robot::setParticlePlotLimit(unsigned int particle_plot_limit)
{
#ifdef USE_OPENRAVE
    viewer_->setParticlePlotLimit(particle_plot_limit);
#endif
}

}
