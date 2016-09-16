#include <robot_headers/Homecare/Homecare.hpp>

namespace frapu
{
Homecare::Homecare(std::string robotFile, std::string configFile):
    Robot(robotFile, configFile),
    lowerStateLimits_(),
    upperStateLimits_(),
    lowerControlLimits_(),
    upperControlLimits_()
{
    lowerStateLimits_.push_back(-1.0);
    lowerStateLimits_.push_back(-1.0);
    upperStateLimits_.push_back(1.0);
    upperStateLimits_.push_back(1.0);

    lowerControlLimits_.push_back(1.0);
    upperControlLimits_.push_back(5.0);
}

std::string Homecare::getName() const {
    std::string name = "Homecare";
    return name;
}

bool Homecare::propagateState(const frapu::RobotStateSharedPtr& state,
                              const frapu::ActionSharedPtr& action,
                              double duration,
                              double simulationStepSize,
                              frapu::RobotStateSharedPtr& result)
{
//TODO
}

bool Homecare::propagateState(const frapu::RobotStateSharedPtr& state,
                              const frapu::ActionSharedPtr& action,
                              const std::vector<double> controlError,
                              double duration,
                              double simulationStepSize,
                              frapu::RobotStateSharedPtr& result)
{
//TODO
}


bool Homecare::makeStateSpace()
{
    stateSpace_ = std::make_shared<frapu::VectorStateSpace>(4);
    frapu::StateLimitsSharedPtr stateLimits =
        std::make_shared<frapu::VectorStateLimits>(lowerStateLimits_, upperStateLimits_);
    stateSpace_->setStateLimits(stateLimits);
}

bool Homecare::makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo)
{
    actionSpace_ = std::make_shared<frapu::DiscreteVectorActionSpace>(actionSpaceInfo);
    unsigned int numDimensions = 1;
    actionSpace_->setNumDimensions(numDimensions);
    frapu::ActionLimitsSharedPtr actionLimits =
        std::make_shared<frapu::VectorActionLimits>(lowerControlLimits_, upperControlLimits_);
    actionSpace_->setActionLimits(actionLimits);
    return true;
}

bool Homecare::makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<frapu::DiscreteObservationSpace>(observationSpaceInfo);
    observationSpace_->setDimension(6);
    std::vector<std::vector<double>> observations;

    // Get the observations using a serializer

    static_cast<frapu::DiscreteObservationSpace*>(observationSpace_.get())->addObservations(observations);
    return true;
}

bool Homecare::getObservation(const frapu::RobotStateSharedPtr& state,
                              frapu::ObservationSharedPtr& observation) const
{
    //TODO
}

bool Homecare::getObservation(const frapu::RobotStateSharedPtr& state,
                              std::vector<double>& observationError,
                              frapu::ObservationSharedPtr& observation) const
{
    //TODO
}

void Homecare::createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
        std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const
{
    //TODO
}

int Homecare::getDOF() const
{
    //TODO
}

void Homecare::makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
        const frapu::RobotStateSharedPtr& collidingState,
        frapu::RobotStateSharedPtr& nextState)
{
    //TODO
}

void Homecare::getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                        const frapu::ActionSharedPtr& control,
                                        double& duration,
                                        std::vector<Eigen::MatrixXd>& matrices) const
{
//TODO
}

void Homecare::getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{
//TODO
}

bool Homecare::isTerminal(const frapu::RobotStateSharedPtr& state) const
{
//TODO
}

double Homecare::distanceGoal(const frapu::RobotStateSharedPtr& state) const
{
//TODO
}

void Homecare::makeProcessDistribution(Eigen::MatrixXd& mean,
                                       Eigen::MatrixXd& covariance_matrix)
{
//TODO
}

void Homecare::makeObservationDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covariance_matrix)
{
//TODO
}

void Homecare::transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
        frapu::ObservationSharedPtr& res) const
{

}

void Homecare::updateViewer(const frapu::RobotStateSharedPtr& state,
                            std::vector<frapu::RobotStateSharedPtr>& particles,
                            std::vector<std::vector<double>>& particleColors)
{
//TODO
}

frapu::RobotStateSharedPtr Homecare::sampleInitialState() const
{
//TODO
}

frapu::HeuristicFunctionSharedPtr Homecare::makeHeuristicFunction() const
{
//TODO
}

}
