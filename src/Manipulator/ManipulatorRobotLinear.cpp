#include <robot_headers/Manipulator/ManipulatorRobotLinear.hpp>


namespace frapu
{
ManipulatorRobotLinear::ManipulatorRobotLinear(std::string robotFile, std::string configFile):
    frapu::ManipulatorRobot(robotFile, configFile)
{
    /**std::vector<double> lowerStateLimits(lowerStateLimits_.size() / 2);
    std::vector<double> upperStateLimits(upperStateLimits_.size() / 2);
    for (size_t i = 0; i < lowerStateLimits.size(); i++) {
        lowerStateLimits[i] = lowerStateLimits_[i];
        upperStateLimits[i] = upperStateLimits_[i];
    }

    lowerStateLimits_ = lowerStateLimits;
    upperStateLimits_ = upperStateLimits;*/
    lowerStateLimits_ = std::vector<double>(lowerStateLimits_.size() / 2);
    upperStateLimits_ = std::vector<double>(upperStateLimits_.size() / 2);
    lowerControlLimits_ = std::vector<double>(lowerStateLimits_.size());
    upperControlLimits_ = std::vector<double>(upperStateLimits_.size());
    for (size_t i = 0; i < lowerStateLimits_.size(); i++) {
        lowerStateLimits_[i] = -3.14;
        upperStateLimits_[i] = 3.14;
        lowerControlLimits_[i] = -0.1;
        upperControlLimits_[i] = 0.1;
    }
    
    propagator_ = std::make_shared<frapu::ManipulatorPropagatorLinear>();

}

std::string ManipulatorRobotLinear::getName() const {
    std::string name = "ManipulatorLinear";
    return name;
}

void ManipulatorRobotLinear::getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
        const frapu::ActionSharedPtr& control,
        double& duration,
        std::vector<Eigen::MatrixXd>& matrices) const
{
    //A_B_V_H_W
    matrices = std::vector<Eigen::MatrixXd>(5);
    unsigned int stateSize = static_cast<frapu::VectorState*>(state.get())->asVector().size();
    for (size_t i = 0; i < 5; i++) {
        matrices[i] = Eigen::MatrixXd::Identity(stateSize, stateSize);
    }
}

void ManipulatorRobotLinear::makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
        const frapu::RobotStateSharedPtr& collidingState,
        frapu::RobotStateSharedPtr& nextState)
{
    nextState = previousState;
}

bool ManipulatorRobotLinear::getObservation(const frapu::RobotStateSharedPtr& state,
        frapu::ObservationSharedPtr& observation) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    observation = std::make_shared<frapu::VectorObservation>(stateVec);
}

bool ManipulatorRobotLinear::getObservation(const frapu::RobotStateSharedPtr& state,
        std::vector<double>& observationError,
        frapu::ObservationSharedPtr& observation) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> observationVec(stateVec.size());
    for (size_t i = 0; i < stateVec.size(); i++) {
        observationVec[i] = stateVec[i] + observationError[i];
    }

    observation = std::make_shared<frapu::VectorObservation>(observationVec);
}

bool ManipulatorRobotLinear::makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<frapu::ContinuousObservationSpace>(observationSpaceInfo);
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    unsigned int observationSpaceDimension = lowerStateLimits_.size();
    cout << "obs dim " << observationSpaceDimension << endl;
    observationSpace_->setDimension(observationSpaceDimension);
    static_cast<frapu::ContinuousObservationSpace*>(observationSpace_.get())->setLimits(lowerStateLimits_,
            upperStateLimits_);


}

void ManipulatorRobotLinear::transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
        frapu::ObservationSharedPtr& res) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> observationVec = stateVec;
    res = std::make_shared<frapu::VectorObservation>(observationVec);
}

void ManipulatorRobotLinear::getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{
    unsigned int numDimensions =
        static_cast<frapu::VectorStateSpace*>(stateSpace_.get())->getNumDimensions();
    H = Eigen::MatrixXd::Identity(numDimensions, numDimensions);
    W = Eigen::MatrixXd::Identity(numDimensions, numDimensions);
 
}

void ManipulatorRobotLinear::updateViewer(const frapu::RobotStateSharedPtr& state,
        std::vector<frapu::RobotStateSharedPtr>& particles,
        std::vector<std::vector<double>>& particleColors)
{
#ifdef USE_OPENRAVE
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> joint_values;
    std::vector<double> joint_velocities;
    std::vector<std::vector<double>> particle_joint_values;
    for (size_t i = 0; i < stateVec.size(); i++) {
        joint_values.push_back(stateVec[i]);
        joint_velocities.push_back(0);
    }

    for (size_t i = 0; i < particles.size(); i++) {
        std::vector<double> particle;
        std::vector<double> particleVec = static_cast<const frapu::VectorState*>(particles[i].get())->asVector();
        for (size_t j = 0; j < stateVec.size(); j++) {
            particle.push_back(particleVec[j]);
        }
        particle_joint_values.push_back(particle);

    }

    viewer_->updateRobotValues(joint_values,
                               joint_velocities,
                               particle_joint_values,
                               particleColors,
                               nullptr);
#endif

}

frapu::RobotStateSharedPtr ManipulatorRobotLinear::sampleInitialState() const
{
    std::vector<double> initStateVec(lowerStateLimits_.size(), 0);
    frapu::RobotStateSharedPtr initState(new frapu::VectorState(initStateVec));
    return initState;
}

void ManipulatorRobotLinear::setNewtonModel() {
    
}

void ManipulatorRobotLinear::setGravityConstant(double gravity_constant) {
    
}

}
