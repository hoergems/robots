#include <robot_headers/AUV/auv.hpp>

namespace frapu
{
AUV::AUV(std::string robotFile, std::string configFile):
    Robot(robotFile, configFile),
    dim_x_(0.0),
    dim_y_(0.0),
    dim_z_(0.0),
    initialState_()
{
    
    serializer_ = std::make_shared<frapu::AUVSerializer>();
    propagator_ = std::make_shared<frapu::AUVPropagator>();
    dim_x_ = 0.005;
    dim_y_ = 0.005;
    dim_z_ = 0.005;

    //make the state limits
    lowerStateLimits_.clear();
    upperStateLimits_.clear();

    lowerStateLimits_.push_back(-1.0);
    lowerStateLimits_.push_back(-1.0);

    upperStateLimits_.push_back(1.0);
    upperStateLimits_.push_back(1.0);

    //make the control limits
    lowerControlLimits_.clear();
    upperControlLimits_.clear();

    lowerControlLimits_.push_back(1.0);
    upperControlLimits_.push_back(5.0);
    std::ifstream inputFile(configFile);
    initialState_ = static_cast<frapu::AUVSerializer *>(serializer_.get())->loadInitalState(inputFile);
}

void AUV::setupHeuristic(frapu::RewardModelSharedPtr &rewardModel) {
    frapu::PathPlannerSharedPtr pathPlanner;
    heuristic_ = std::make_shared<frapu::RRTHeuristic>(pathPlanner);
}

frapu::RobotStateSharedPtr AUV::sampleInitialState() const {
    return initialState_;
}

void AUV::createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
                                      std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const
{
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    double x = stateVec[0];
    double y = stateVec[1];
    fcl::Vec3f trans_vec(x, y, 0.001);
    fcl::Matrix3f rot_matrix(1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0);
    fcl::Transform3f trans(rot_matrix, trans_vec);
    fcl::AABB link_aabb(fcl::Vec3f(-0.01,
                                   -0.01,
                                   -0.01),
                        fcl::Vec3f(0.01,
                                   0.01,
                                   0.01));
    fcl::Box* box = new fcl::Box();
    fcl::Transform3f box_tf;
    fcl::constructBox(link_aabb, trans, *box, box_tf);
    frapu::CollisionObjectSharedPtr coll_obj = std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
    collision_objects.push_back(coll_obj);
}

bool AUV::makeStateSpace()
{
    stateSpace_ = std::make_shared<frapu::VectorStateSpace>(2);
    frapu::StateLimitsSharedPtr stateLimits =
        std::make_shared<frapu::VectorStateLimits>(lowerStateLimits_, upperStateLimits_);
    stateSpace_->setStateLimits(stateLimits);
}

bool AUV::makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo)
{
    actionSpace_ = std::make_shared<frapu::DiscreteVectorActionSpace>(actionSpaceInfo);
    unsigned int numDimensions = 1;
    actionSpace_->setNumDimensions(numDimensions);
    frapu::ActionLimitsSharedPtr actionLimits =
        std::make_shared<frapu::VectorActionLimits>(lowerControlLimits_, upperControlLimits_);
    actionSpace_->setActionLimits(actionLimits);
}

bool AUV::makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<frapu::DiscreteObservationSpace>(observationSpaceInfo);
    observationSpace_->setDimension(2);
    std::vector<std::vector<double>> observations;

    // Get the observations using a serializer

    static_cast<frapu::DiscreteObservationSpace*>(observationSpace_.get())->addObservations(observations);
    return true;
}

bool AUV::getObservation(const frapu::RobotStateSharedPtr& state,
                         std::vector<double>& observationError,
                         frapu::ObservationSharedPtr& observation) const
{
    return getObservation(state, observation);
}

bool AUV::getObservation(const frapu::RobotStateSharedPtr& state,
                         frapu::ObservationSharedPtr& observation) const
{
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    std::vector<frapu::ObstacleSharedPtr> obstacles;
    environmentInfo_->scene->getObstacles(obstacles);
    std::vector<double> observationVec;
    std::vector<frapu::CollisionObjectSharedPtr> collisionObjects;
    createRobotCollisionObjects(state, collisionObjects);
    for (auto & obstacle : obstacles) {
        if (obstacle->getTerrain()->isObservable()) {
            if (obstacle->inCollision(collisionObjects)) {
                observationVec = stateVec;
                return true;
            }
        }
    }

    observationVec.clear();
    observationVec.push_back(-100);
    observationVec.push_back(-100);
    observation = std::make_shared<frapu::VectorObservation>(observationVec);
    return true;
}

double AUV::calcLikelihood(const frapu::RobotStateSharedPtr& state,
                           const frapu::ObservationSharedPtr& observation) const
{
    frapu::ObservationSharedPtr observationState;
    transformToObservationSpace(state, observationState);
    std::vector<double> observationVec =
        static_cast<frapu::VectorObservation*>(observationState.get())->asVector();
    std::vector<double> stateVec =
        static_cast<frapu::VectorState*>(state.get())->asVector();
    bool isSame = true;
    for (size_t i = 0; i < observationVec.size(); i++) {
        if (stateVec[i] != observationVec[i]) {
            isSame = false;
        }
    }

    if (isSame) {
        return 1.0;
    }

    return 0.0;
}

void AUV::transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
                                      frapu::ObservationSharedPtr& res) const
{
    getObservation(state, res);
}

int AUV::getDOF() const
{
    return 2;
}

void AUV::makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
                                      const frapu::RobotStateSharedPtr& collidingState,
                                      frapu::RobotStateSharedPtr& nextState)
{
    nextState = previousState;
}

bool AUV::isTerminal(const frapu::RobotStateSharedPtr& state) const
{
    double dist = distanceGoal(state);
    if (dist < goal_radius_) {
        return true;
    }

    return false;
}

double AUV::distanceGoal(const frapu::RobotStateSharedPtr& state) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    assert(goal_position_.size() != 0 && "DubinRobot: No goal area set. Cannot calculate distance!");
    double x = stateVec[0];
    double y = stateVec[1];

    double dist = std::pow(goal_position_[0] - x, 2);
    dist += std::pow(goal_position_[1] - y, 2);
    return std::sqrt(dist);
}

void AUV::setGravityConstant(double gravity_constant)
{

}

void AUV::getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
                                       Eigen::MatrixXd& H,
                                       Eigen::MatrixXd& W) const
{

}

void AUV::getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                   const frapu::ActionSharedPtr& control,
                                   double& duration,
                                   std::vector<Eigen::MatrixXd>& matrices) const
{

}

void AUV::makeProcessDistribution(Eigen::MatrixXd& mean,
                                  Eigen::MatrixXd& covariance_matrix,
                                  unsigned long seed)
{
    process_distribution_ = std::make_shared<Eigen::WeightedDiscreteDistribution<double>>();
    std::vector<std::pair<std::vector<double>, double>> elements;
    std::vector<double> elem0( { -0.01, 0.0});
    std::vector<double> elem1( {0.0, 0.0});
    std::vector<double> elem2( {0.01, 0.0});
    elements.push_back(std::make_pair(elem0, 0.1));
    elements.push_back(std::make_pair(elem1, 0.8));
    elements.push_back(std::make_pair(elem2, 0.1));
    static_cast<Eigen::WeightedDiscreteDistribution<double> *>(process_distribution_.get())->setElements(elements);
}

void AUV::makeObservationDistribution(Eigen::MatrixXd& mean,
                                      Eigen::MatrixXd& covariance_matrix,
                                      unsigned long seed)
{
    observation_distribution_ = std::make_shared<Eigen::WeightedDiscreteDistribution<double>>();
}

void AUV::updateRobot(const frapu::RobotStateSharedPtr& state)
{
    cout << "UPDATE" << endl;    
}

void AUV::updateViewer(const frapu::RobotStateSharedPtr& state,
                       std::vector<frapu::RobotStateSharedPtr>& particles,
                       std::vector<std::vector<double>>& particleColors)
{
#ifdef USE_OPENRAVE
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<std::string> names;
    std::vector<std::vector<double>> dims;
    std::vector<std::vector<double>> colors;
    std::string name = "auv";
    names.push_back(name);
    std::vector<double> main_dims( {stateVec[0], stateVec[1], 0.001, dim_x_, dim_y_, dim_z_, 0.0});
    dims.push_back(main_dims);
    std::vector<double> main_color( {1.0, 0.0, 0.0, 0.5});
    colors.push_back(main_color);
    for (size_t i = 0; i < particles.size(); i++) {
        std::string p_name = "particle_auv" + std::to_string(i);
        names.push_back(p_name);
	std::vector<double> particle = static_cast<const frapu::VectorState *>(particles[i].get())->asVector();
        std::vector<double> p_dims( {particle[0],
                                     particle[1],
                                     0.001,
                                     dim_x_,
                                     dim_y_,
                                     dim_z_,
                                     0.0
                                    });
        dims.push_back(p_dims);
        //std::vector<double> c({0.0, 1.0, 0.0, 0.5});
        colors.push_back(particleColors[i]);
    }

    viewer_->addBoxes(names, dims, colors);
#endif
}

}
