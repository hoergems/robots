#include "include/DubinRobot.hpp"

namespace shared
{

DubinRobot::DubinRobot(std::string robot_file):
    Robot(robot_file),
    dim_x_(0.0),
    dim_y_(0.0),
    dim_z_(0.0),
    d_(0.0),
    beacons_()
{
    //Dimensions
    dim_x_ = 0.06;
    dim_y_ = 0.035;
    dim_z_ = 0.005;

    //Distance between axels
    d_ = 0.11;

    propagator_ = std::make_shared<shared::DubinPropagator>();
    static_cast<shared::DubinPropagator*>(propagator_.get())->setD(d_);

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
    shared::Beacon b0(-0.7, 0.7);
    shared::Beacon b1(0.7, -0.7);
    beacons_ = std::vector<shared::Beacon>( {b0, b1});
}

void DubinRobot::createRobotCollisionObjects(const std::vector<double>& state,
        std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const
{
    double x = state[0];
    double y = state[1];
    double theta = state[2];

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

bool DubinRobot::makeActionSpace(bool normalizedActionSpace) {
    actionSpace_ = std::make_shared<shared::DiscreteActionSpace>(normalizedActionSpace);
    unsigned int numDimensions = 2;
    actionSpace_->setNumDimensions(numDimensions);
    actionSpace_->setActionLimits(lowerControlLimits_, upperControlLimits_);
}

bool DubinRobot::makeObservationSpace(const shared::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<shared::ContinuousObservationSpace>(observationSpaceInfo);
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    if (observationSpaceInfo.observationType == "linear") {
        observationSpace_->setDimension(getStateSpaceDimension());
        getStateLimits(lowerLimits, upperLimits);
        static_cast<shared::ContinuousObservationSpace*>(observationSpace_.get())->setLimits(lowerLimits,
                upperLimits);
    } else {
        observationSpace_->setDimension(3);
        lowerLimits = std::vector<double>( {0.0, 0.0, -1.2});
        upperLimits = std::vector<double>( {1.0, 1.0, 1.2});
        static_cast<shared::ContinuousObservationSpace*>(observationSpace_.get())->setLimits(lowerLimits,
                upperLimits);
    }
}

bool DubinRobot::getObservation(std::vector<double>& state, std::vector<double>& observationError, std::vector<double>& observation) const
{
    std::vector<double> res;
    transformToObservationSpace(state, res);
    observation = std::vector<double>(observationSpace_->getDimension());
    observation[0] = res[0] + observationError[0];
    observation[1] = res[1] + observationError[1];
    observation[2] = res[2] + observationError[2];
}

bool DubinRobot::getObservation(std::vector<double>& state, std::vector<double>& observation) const
{
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        observation.clear();
        Eigen::MatrixXd sample = observation_distribution_->samples(1);
        for (size_t i = 0; i < state.size(); i++) {
            observation.push_back(state[i] + sample(i, 0));
        }
    } else {
        observation = std::vector<double>(3);
        unsigned int observationSpaceDimension = observationSpace_->getDimension();
        Eigen::MatrixXd sample = observation_distribution_->samples(1);

        observation[0] = sample(0, 0) + 1.0 / (std::pow(state[0] - beacons_[0].x_, 2) + std::pow(state[1] - beacons_[0].y_, 2) + 1.0);
        observation[1] = sample(1, 0) + 1.0 / (std::pow(state[0] - beacons_[1].x_, 2) + std::pow(state[1] - beacons_[1].y_, 2) + 1.0);
        observation[2] = state[3] + sample(2, 0);

    }

    return true;
}

void DubinRobot::transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const
{
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        res = state;
    } else {
        res = std::vector<double>(3);
        res[0] = 1.0 / (std::pow(state[0] - beacons_[0].x_, 2) + std::pow(state[1] - beacons_[0].y_, 2) + 1.0);
        res[1] = 1.0 / (std::pow(state[0] - beacons_[1].x_, 2) + std::pow(state[1] - beacons_[1].y_, 2) + 1.0);
        res[2] = state[3];
    }
}

int DubinRobot::getStateSpaceDimension() const
{
    return 4;
}

int DubinRobot::getDOF() const
{
    return 4;
}

void DubinRobot::makeNextStateAfterCollision(std::vector<double>& previous_state,
        std::vector<double>& colliding_state,
        std::vector<double>& next_state)
{
    next_state = previous_state;
    next_state[3] = 0.0;
}

bool DubinRobot::isTerminal(std::vector<double>& state) const
{
    double dist = distanceGoal(state);
    if (dist < goal_radius_) {
        return true;
    }

    return false;
}


double DubinRobot::distanceGoal(std::vector<double>& state) const
{
    assert(goal_position_.size() != 0 && "DubinRobot: No goal area set. Cannot calculate distance!");
    double x = state[0];
    double y = state[1];

    double dist = std::pow(goal_position_[0] - x, 2);
    dist += std::pow(goal_position_[1] - y, 2);
    return std::sqrt(dist);
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

void DubinRobot::getLinearObservationDynamics(const std::vector<double>& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{
    if (observationSpace_->getObservationSpaceInfo().observationType == "linear") {
        H = Eigen::MatrixXd::Identity(4, 4);
        W = Eigen::MatrixXd::Identity(4, 4);
    } else {
        getLinearObservationMatrix(state, H);
        W = Eigen::MatrixXd::Identity(3, 3);
    }
}

void DubinRobot::getLinearProcessMatrices(const std::vector<double>& state,
        std::vector<double>& control,
        double& duration,
        std::vector<Eigen::MatrixXd>& matrices) const
{
    Eigen::MatrixXd A(4, 4);
    A << 1, 0, -duration* state[3]*sin(state[2]), duration* cos(state[2]),
      0, 1, duration* state[3]*cos(state[2]), duration* sin(state[2]),
      0, 0, 1, duration* tan(control[1]) / d_,
      0, 0, 0, 1;

    Eigen::MatrixXd B(4, 2);
    B << 0, 0,
      0, 0,
      0, duration* state[3]*(std::pow(tan(control[1]), 2) + 1) / d_,
      duration, 0;

    Eigen::MatrixXd V(4, 2);
    V << 0, 0,
      0, 0,
      0, duration* state[3]*(std::pow(tan(control[1]), 2) + 1) / d_,
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
        getLinearObservationMatrix(state, H);
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
    setStateCovarianceMatrix(process_distribution_->_covar);
}

void DubinRobot::makeObservationDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covariance_matrix,
        unsigned long seed)
{   
    observation_distribution_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, covariance_matrix, false, seed);
    setObservationCovarianceMatrix(observation_distribution_->_covar);
}

void DubinRobot::updateViewer(std::vector<double>& state,
                              std::vector<std::vector<double>>& particles,
                              std::vector<std::vector<double>>& particle_colors)
{
#ifdef USE_OPENRAVE
    std::vector<std::string> names;
    std::vector<std::vector<double>> dims;
    std::vector<std::vector<double>> colors;
    std::string name = "dubin";
    names.push_back(name);
    std::vector<double> main_dims( {state[0], state[1], 0.025, dim_x_, dim_y_, dim_z_, state[2]});
    dims.push_back(main_dims);
    std::vector<double> main_color( {1.0, 0.0, 0.0, 0.5});
    colors.push_back(main_color);
    for (size_t i = 0; i < particles.size(); i++) {
        std::string p_name = "particle_dubin" + std::to_string(i);
        names.push_back(p_name);

        std::vector<double> p_dims( {particles[i][0],
                                     particles[i][1],
                                     0.025,
                                     dim_x_,
                                     dim_y_,
                                     dim_z_,
                                     particles[i][2]
                                    });
        dims.push_back(p_dims);
        //std::vector<double> c({0.0, 1.0, 0.0, 0.5});
        colors.push_back(particle_colors[i]);
    }

    viewer_->addBoxes(names, dims, colors);


#endif
}

}
