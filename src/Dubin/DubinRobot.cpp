#include <robot_headers/Dubin/DubinRobot.hpp>

namespace frapu
{

DubinRobot::DubinRobot(std::string robotFile, std::string configFile):
    Robot(robotFile, configFile),
    dim_x_(0.0),
    dim_y_(0.0),
    dim_z_(0.0),
    d_(0.0),
    beacons_(),
    initialState_(nullptr)
{
    //Dimensions
    dim_x_ = 0.06;
    dim_y_ = 0.035;
    dim_z_ = 0.005;

    //Distance between axels
    d_ = 0.11;

    serializer_ = std::make_shared<frapu::DubinSerializer>();
    propagator_ = std::make_shared<frapu::DubinPropagator>();
    static_cast<frapu::DubinPropagator*>(propagator_.get())->setD(d_);

    //make the state limits
    lowerStateLimits_.clear();
    upperStateLimits_.clear();

    lowerStateLimits_.push_back(-1.0);
    lowerStateLimits_.push_back(-1.0);
    lowerStateLimits_.push_back(-3.14);
    lowerStateLimits_.push_back(-0.2);

    upperStateLimits_.push_back(1.0);
    upperStateLimits_.push_back(1.0);
    upperStateLimits_.push_back(3.14);
    upperStateLimits_.push_back(0.2);

    //make the control limits
    lowerControlLimits_.clear();
    upperControlLimits_.clear();

    lowerControlLimits_.push_back(0.0);
    lowerControlLimits_.push_back(-1.0);

    upperControlLimits_.push_back(1.0);
    upperControlLimits_.push_back(1.0);

    // put the beacons in the evironment
    Beacon b0(-0.7, 0.7);
    Beacon b1(0.7, -0.7);
    beacons_ = std::vector<Beacon>( {b0, b1});

    std::ifstream input(configFile);
    initialState_ = static_cast<frapu::DubinSerializer*>(serializer_.get())->loadInitalState(input);
}

frapu::HeuristicFunctionSharedPtr DubinRobot::makeHeuristicFunction() const
{
    frapu::HeuristicFunctionSharedPtr heuristicFunction = std::make_shared<RRTHeuristicFunction>();
    auto terminalFunction = std::bind(&DubinRobot::isTerminal, this, std::placeholders::_1);
    heuristicFunction->setTerminalFunction(terminalFunction);
    return heuristicFunction;
}

frapu::RobotStateSharedPtr DubinRobot::sampleInitialState() const
{    
    return initialState_;
}

void DubinRobot::createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
        std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const
{
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    double x = stateVec[0];
    double y = stateVec[1];
    double theta = stateVec[2];

    fcl::Matrix3f rot_matrix(cos(theta), -sin(theta), 0.0,
                             sin(theta), cos(theta), 0.0,
                             0.0, 0.0, 1.0);
    fcl::Vec3f trans_vec(x, y, 0.01 + dim_z_ / 2.0);
    fcl::Transform3f trans(rot_matrix, trans_vec);
    fcl::AABB link_aabb(fcl::Vec3f(-dim_x_ / 2.0,
                                   -dim_y_ / 2.0,
                                   -dim_z_ / 2.0),
                        fcl::Vec3f(dim_x_ / 2.0,
                                   dim_y_ / 2.0,
                                   dim_z_ / 2.0));
    fcl::Box* box = new fcl::Box();
    fcl::Transform3f box_tf;
    fcl::constructBox(link_aabb, trans, *box, box_tf);
    std::shared_ptr<fcl::CollisionObject> coll_obj =
        std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
    collision_objects.push_back(coll_obj);
}

bool DubinRobot::makeStateSpace()
{
    stateSpace_ = std::make_shared<frapu::VectorStateSpace>(4);
    frapu::StateLimitsSharedPtr stateLimits =
        std::make_shared<frapu::VectorStateLimits>(lowerStateLimits_, upperStateLimits_);
    stateSpace_->setStateLimits(stateLimits);
}

void DubinRobot::makeGoal()
{
    goal_ = std::make_shared<frapu::SphereGoal>(goal_position_, goal_radius_);
}

bool DubinRobot::makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo)
{
    if (actionSpaceInfo.type == "continuous") {
        actionSpace_ = std::make_shared<frapu::ContinuousVectorActionSpace>(actionSpaceInfo);
    } else {
        actionSpace_ = std::make_shared<frapu::DiscreteVectorActionSpace>(actionSpaceInfo);
    }

    unsigned int numDimensions = 2;
    actionSpace_->setNumDimensions(numDimensions);

    frapu::ActionLimitsSharedPtr actionLimits =
        std::make_shared<frapu::VectorActionLimits>(lowerControlLimits_, upperControlLimits_);
    actionSpace_->setActionLimits(actionLimits);
}

bool DubinRobot::makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<frapu::ContinuousObservationSpace>(observationSpaceInfo);
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    if (observationSpaceInfo.observationType == "linear") {
        observationSpace_->setDimension(4);
        static_cast<frapu::ContinuousObservationSpace*>(observationSpace_.get())->setLimits(lowerStateLimits_,
                upperStateLimits_);
    } else {
        observationSpace_->setDimension(3);
        lowerLimits = std::vector<double>( {0.0, 0.0, -1.2});
        upperLimits = std::vector<double>( {1.0, 1.0, 1.2});
        static_cast<frapu::ContinuousObservationSpace*>(observationSpace_.get())->setLimits(lowerLimits,
                upperLimits);
    }
}

bool DubinRobot::getObservation(const frapu::RobotStateSharedPtr& state,
                                std::vector<double>& observationError,
                                frapu::ObservationSharedPtr& observation) const
{
    transformToObservationSpace(state, observation);
    std::vector<double> observationVec =
        static_cast<frapu::VectorObservation*>(observation.get())->asVector();
    for (size_t i = 0; i < observationError.size(); i++) {
        observationVec[i] += observationError[i];
    }

    observation = std::make_shared<frapu::VectorObservation>(observationVec);
}

bool DubinRobot::getObservation(const frapu::RobotStateSharedPtr& state,
                                frapu::ObservationSharedPtr& observation) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> observationVec;
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        observationVec = std::vector<double>(stateVec.size());
        Eigen::MatrixXd sample = observation_distribution_->samples(1);
        for (size_t i = 0; i < stateVec.size(); i++) {
            observationVec[i] = stateVec[i] + sample(i, 0);
        }
    } else {
        observationVec = std::vector<double>(3);
        unsigned int observationSpaceDimension = observationSpace_->getDimension();
        Eigen::MatrixXd sample = observation_distribution_->samples(1);

        observationVec[0] = sample(0, 0) + 1.0 / (std::pow(stateVec[0] - beacons_[0].x_, 2) + std::pow(stateVec[1] - beacons_[0].y_, 2) + 1.0);
        observationVec[1] = sample(1, 0) + 1.0 / (std::pow(stateVec[0] - beacons_[1].x_, 2) + std::pow(stateVec[1] - beacons_[1].y_, 2) + 1.0);
        observationVec[2] = stateVec[3] + sample(2, 0);

    }

    observation = std::make_shared<frapu::VectorObservation>(observationVec);
    return true;
}

void DubinRobot::transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
        frapu::ObservationSharedPtr& res) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> observationVec;
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        observationVec = stateVec;
    } else {
        observationVec = std::vector<double>(3);
        observationVec[0] = 1.0 / (std::pow(stateVec[0] - beacons_[0].x_, 2) + std::pow(stateVec[1] - beacons_[0].y_, 2) + 1.0);
        observationVec[1] = 1.0 / (std::pow(stateVec[0] - beacons_[1].x_, 2) + std::pow(stateVec[1] - beacons_[1].y_, 2) + 1.0);
        observationVec[2] = stateVec[3];
    }

    res = std::make_shared<frapu::VectorObservation>(observationVec);
}

int DubinRobot::getDOF() const
{
    return 4;
}

void DubinRobot::makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
        const frapu::RobotStateSharedPtr& collidingState,
        frapu::RobotStateSharedPtr& nextState)
{
    std::vector<double> previousStateVec = static_cast<frapu::VectorState*>(previousState.get())->asVector();
    std::vector<double> nextStateVec = previousStateVec;
    nextStateVec[3] = 0.0;
    nextState = std::make_shared<frapu::VectorState>(nextStateVec);
}

bool DubinRobot::isTerminal(const frapu::RobotStateSharedPtr& state) const
{
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    if (stateVec.size() > 4) {
	frapu::ERROR("state vec size " + std::to_string(stateVec.size()));
    }
    
    std::vector<double> sVec(3);
    sVec[0] = stateVec[0];
    sVec[1] = stateVec[1];
    sVec[2] = 0.0;
    
    return static_cast<frapu::SphereGoal*>(goal_.get())->isSatisfied(sVec);
}


double DubinRobot::distanceGoal(const frapu::RobotStateSharedPtr& state) const
{
    std::vector<double> stateVec = static_cast<const frapu::VectorState*>(state.get())->asVector();
    std::vector<double> sVec(3);
    sVec[0] = stateVec[0];
    sVec[1] = stateVec[1];
    sVec[2] = 0.0;
    return static_cast<frapu::SphereGoal*>(goal_.get())->distanceCenter(sVec);
    /**assert(goal_position_.size() != 0 && "DubinRobot: No goal area set. Cannot calculate distance!");
    double x = stateVec[0];
    double y = stateVec[1];

    double dist = std::pow(goal_position_[0] - x, 2);
    dist += std::pow(goal_position_[1] - y, 2);
    return std::sqrt(dist);*/
}

void DubinRobot::setGravityConstant(double gravity_constant)
{

}

void DubinRobot::getLinearObservationMatrix(const std::vector<double>& state, Eigen::MatrixXd& H) const
{
    H = Eigen::MatrixXd(3, 4);

    H(0, 0) = 1.0 * (2 * beacons_[0].x_ - 2 * state[0]) / std::pow(std::pow(-beacons_[0].x_ + state[0], 2) + std::pow(-beacons_[0].y_ + state[1], 2) + 1.0, 2);
    H(0, 1) = 1.0 * (2 * beacons_[0].y_ - 2 * state[1]) / std::pow(std::pow(-beacons_[0].x_ + state[0], 2) + std::pow(-beacons_[0].y_ + state[1], 2) + 1.0, 2);
    H(0, 2) = 0.0;
    H(0, 3) = 0.0;
    H(1, 0) = 1.0 * (2 * beacons_[1].x_ - 2 * state[0]) / std::pow(std::pow(-beacons_[1].x_ + state[0], 2) + std::pow(-beacons_[1].y_ + state[1], 2) + 1.0, 2);
    H(1, 1) = 1.0 * (2 * beacons_[1].y_ - 2 * state[1]) / std::pow(std::pow(-beacons_[1].x_ + state[0], 2) + std::pow(-beacons_[1].y_ + state[1], 2) + 1.0, 2);
    H(1, 2) = 0.0;
    H(1, 3) = 0.0;
    H(2, 0) = 0.0;
    H(2, 1) = 0.0;
    H(2, 2) = 0.0;
    H(2, 3) = 1.0;
}

void DubinRobot::getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        H = Eigen::MatrixXd::Identity(4, 4);
        W = Eigen::MatrixXd::Identity(4, 4);
    } else {
        getLinearObservationMatrix(stateVec, H);
        W = Eigen::MatrixXd::Identity(3, 3);
    }
}

void DubinRobot::getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
        const frapu::ActionSharedPtr& control,
        double& duration,
        std::vector<Eigen::MatrixXd>& matrices) const
{
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<double> controlVec = static_cast<frapu::VectorAction*>(control.get())->asVector();
    Eigen::MatrixXd A(4, 4);
    A << 1, 0, -duration* stateVec[3]*sin(stateVec[2]), duration* cos(stateVec[2]),
      0, 1, duration* stateVec[3]*cos(stateVec[2]), duration* sin(stateVec[2]),
      0, 0, 1, duration* tan(controlVec[1]) / d_,
      0, 0, 0, 1;

    Eigen::MatrixXd B(4, 2);
    B << 0, 0,
      0, 0,
      0, duration* stateVec[3]*(std::pow(tan(controlVec[1]), 2) + 1) / d_,
      duration, 0;

    Eigen::MatrixXd V(4, 2);
    V << 0, 0,
      0, 0,
      0, duration* stateVec[3]*(std::pow(tan(controlVec[1]), 2) + 1) / d_,
      duration, 0;

    matrices.push_back(A);
    matrices.push_back(B);
    matrices.push_back(V);
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(4, 4);
        matrices.push_back(H);
        matrices.push_back(W);
    } else {
        Eigen::MatrixXd H;
        getLinearObservationMatrix(stateVec, H);
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(3, 3);
        matrices.push_back(H);
        matrices.push_back(W);
    }

}

void DubinRobot::makeProcessDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covariance_matrix,
        unsigned long seed)
{
    process_distribution_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, covariance_matrix, false, seed);
}

void DubinRobot::makeObservationDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covariance_matrix,
        unsigned long seed)
{
    observation_distribution_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, covariance_matrix, false, seed);
}

void DubinRobot::updateViewer(const frapu::RobotStateSharedPtr& state,
                              std::vector<frapu::RobotStateSharedPtr>& particles,
                              std::vector<std::vector<double>>& particleColors)
{
#ifdef USE_OPENRAVE
    std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
    std::vector<std::string> names;
    std::vector<std::vector<double>> dims;
    std::vector<std::vector<double>> colors;
    std::string name = "dubin";
    names.push_back(name);
    std::vector<double> main_dims( {stateVec[0], stateVec[1], 0.025, dim_x_, dim_y_, dim_z_, stateVec[2]});
    dims.push_back(main_dims);
    std::vector<double> main_color( {1.0, 0.0, 0.0, 0.5});
    colors.push_back(main_color);
    for (size_t i = 0; i < particles.size(); i++) {
        std::string p_name = "particle_dubin" + std::to_string(i);
        names.push_back(p_name);
        std::vector<double> particle = static_cast<const frapu::VectorState*>(particles[i].get())->asVector();
        std::vector<double> p_dims( {particle[0],
                                     particle[1],
                                     0.025,
                                     dim_x_,
                                     dim_y_,
                                     dim_z_,
                                     particle[2]
                                    });
        dims.push_back(p_dims);
        colors.push_back(particleColors[i]);
    }

    viewer_->addBoxes(names, dims, colors);


#endif
}

}
