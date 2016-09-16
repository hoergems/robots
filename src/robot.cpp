#include <robot_headers/robot.hpp>

using std::cout;
using std::endl;

namespace frapu
{

Robot::Robot(std::string robotFile, std::string configFile):
    InterfaceBase(),
    robot_file_(robotFile),
    constraints_enforced_(true),
    propagator_(nullptr),
    viewer_(nullptr),
    goal_position_(),
    goal_radius_(),
    process_distribution_(nullptr),
    observation_distribution_(nullptr),
    environmentInfo_(nullptr),
    stateSpace_(nullptr),
    actionSpace_(nullptr),
    observationSpace_(nullptr),
    serializer_(nullptr),    
    goal_(nullptr),
    randomEngine_()
{
#ifdef USE_OPENRAVE
    viewer_ = std::make_shared<frapu::ViewerInterface>();
#endif    
}

void Robot::setRandomEngine(std::default_random_engine &randomEngine) {
    randomEngine_ = randomEngine;
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
    //cout << "sample: " << sample << endl;
    for (size_t i = 0; i < controlError.size(); i++) {
        controlError[i] = sample(i, 0);
    }

    return propagateState(state, action, controlError, duration, simulationStepSize, result);
}

bool Robot::isValid(const frapu::RobotStateSharedPtr& state) const
{
    std::vector<frapu::CollisionObjectSharedPtr> collisionObjects;
    std::vector<frapu::ObstacleSharedPtr> obstacles;
    environmentInfo_->scene->getObstacles(obstacles);
    createRobotCollisionObjects(state, collisionObjects);
    
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (!obstacles[i]->getTerrain()->isTraversable()) {
            if (obstacles[i]->inCollision(collisionObjects)) {
                return false;
            }
        }
    }

    return true;
}

void Robot::setControlDuration(double control_duration)
{
    control_duration_ = control_duration;
}

double Robot::getControlDuration() const
{
    return control_duration_;
}

frapu::SerializerSharedPtr Robot::getSerializer() const
{
    return serializer_;
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

double Robot::calcLikelihood(const frapu::RobotStateSharedPtr& state,
                             const frapu::ObservationSharedPtr& observation) const
{
    frapu::ObservationSharedPtr observationState;
    transformToObservationSpace(state, observationState);
    std::vector<double> stateVec =
        static_cast<frapu::VectorObservation*>(observationState.get())->asVector();
    std::vector<double> observationVec =
        static_cast<frapu::VectorObservation*>(observation.get())->asVector();
    double pdf = observation_distribution_->calcPdf(observationVec, stateVec);
    return pdf;
}

frapu::StateSpaceSharedPtr Robot::getStateSpace() const
{
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

void Robot::getGoalArea(std::vector<double> &goalArea) const {
    goalArea.clear();
    for (size_t i = 0; i < goal_position_.size(); i++) { 
	goalArea.push_back(goal_position_[i]);	
    }
    
    goalArea.push_back(goal_radius_);
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

void Robot::setNewtonModel()
{

}

void Robot::setGravityConstant(double gravity_constant)
{

}

void Robot::setEnvironmentInfo(frapu::EnvironmentInfoSharedPtr& environmentInfo)
{    
    if (!environmentInfo) {
	frapu::ERROR("Robot: setSenvironmentInfo: environmentInfo is null!!!");
    }
    environmentInfo_ = environmentInfo;
}

std::vector<frapu::RobotStateSharedPtr> Robot::loadGoalStatesFromFile(std::string &filename) const {
    return serializer_->loadGoalStatesFromFile(filename);
}

void Robot::makeGoal() {
    
}
    
frapu::GoalSharedPtr Robot::getGoal() const {
    return goal_;
}

std::vector<frapu::RobotStateSharedPtr> Robot::getGoalStates() const
{
    return goalStates_;
}

void Robot::setGoalStates(std::vector<frapu::RobotStateSharedPtr> &goalStates) {
    goalStates_ = goalStates;
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
