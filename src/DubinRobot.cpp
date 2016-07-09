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
    dim_x_ = 0.5;
    dim_y_ = 0.3;
    dim_z_ = 0.05;

    //Distance between axels
    d_ = 0.45;

    propagator_ = std::make_shared<shared::DubinPropagator>();
    static_cast<shared::DubinPropagator*>(propagator_.get())->setD(d_);

    //make the state limits
    lowerStateLimits_.clear();
    upperStateLimits_.clear();

    lowerStateLimits_.push_back(-9.0);
    lowerStateLimits_.push_back(-9.0);
    lowerStateLimits_.push_back(-3.14);
    lowerStateLimits_.push_back(-1.2);

    upperStateLimits_.push_back(9.0);
    upperStateLimits_.push_back(9.0);
    upperStateLimits_.push_back(3.14);
    upperStateLimits_.push_back(1.2);

    //make the control limits
    lowerControlLimits_.clear();
    upperControlLimits_.clear();

    lowerControlLimits_.push_back(-50.0);
    lowerControlLimits_.push_back(-0.65);

    upperControlLimits_.push_back(50.0);
    upperControlLimits_.push_back(0.65);

    // put the beacons in the evironment
    shared::Beacon b0(-7.0, 7.0);
    shared::Beacon b1(7.0, -7.0);
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

bool DubinRobot::makeObservationSpace(std::string& observationType)
{    
    observationSpace_ = std::make_shared<shared::ObservationSpace>(observationType);
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    if (observationType == "linear") {
        observationSpace_->setDimension(getStateSpaceDimension());
        getStateLimits(lowerLimits, upperLimits);
        observationSpace_->setLimits(lowerLimits, upperLimits);
    } else {
        observationSpace_->setDimension(3);
        lowerLimits = std::vector<double>( {0.0019, 0.0019, -1.2});
        upperLimits = std::vector<double>( {1.0, 1.0, 1.2});
        observationSpace_->setLimits(lowerLimits, upperLimits);
    }
}

bool DubinRobot::getObservation(std::vector<double> &state, std::vector<double> &observationError, std::vector<double>& observation) const {
    std::vector<double> res;
    transformToObservationSpace(state, res);
    observation = std::vector<double>(observationSpace_->getDimension());
    observation[0] = res[0] + observationError[0];
    observation[1] = res[1] + observationError[1];
    observation[2] = res[2] + observationError[2];
}

bool DubinRobot::getObservation(std::vector<double>& state, std::vector<double>& observation)
{
    if (observationType_ == "linear") {
        observation.clear();
        Eigen::MatrixXd sample = observation_distribution_->samples(1);
        for (size_t i = 0; i < state.size(); i++) {
            observation.push_back(state[i] + sample(i, 0));
        }
    } else {
        observation = std::vector<double>(3);
        unsigned int observationSpaceDimension = observationSpace_->getDimension();
        Eigen::MatrixXd sample = observation_distribution_->samples(1);
        observation[0] = (1.0 / (std::pow(state[0] - beacons_[0].x_, 2) + std::pow(state[1] - beacons_[0].y_, 2) + 1.0)) + sample(0, 0);
        observation[1] = (1.0 / (std::pow(state[0] - beacons_[1].x_, 2) + std::pow(state[1] - beacons_[1].y_, 2) + 1.0)) + sample(1, 0);
        observation[2] = state[3] + sample(2, 0);

    }

    return true;
}

void DubinRobot::transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const
{
    if (observationType_ == "linear") {
        res = state;
    } else {
        res = std::vector<double>(3);
        res[0] = (1.0 / (std::pow(state[0] + 7.0, 2) + std::pow(state[1] - 7.0, 2) + 1.0));
        res[1] = (1.0 / (std::pow(state[0] - 7.0, 2) + std::pow(state[1] + 7.0, 2) + 1.0));
        res[2] = state[3];
    }
}

int DubinRobot::getStateSpaceDimension() const
{
    return 4;
}

int DubinRobot::getControlSpaceDimension() const
{
    return 2;
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
    //cout << "dist: " << std::sqrt(dist) << endl;
    return std::sqrt(dist);
}

void DubinRobot::setGravityConstant(double gravity_constant)
{

}

void DubinRobot::getLinearObservationDynamics(const std::vector<double>& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{
    if (observationType_ == "linear") {
        H = Eigen::MatrixXd::Identity(4, 4);
        W = Eigen::MatrixXd::Identity(4, 4);        
    } else {
	H = Eigen::MatrixXd(3, 4);       
        H(0, 0) = 1.0 * (-2 * state[0] - 14.0) / std::pow(std::pow(state[0] + 7.0, 2) + std::pow(state[1] - 7.0, 2) + 1.0, 2);
        H(0, 1) = 1.0 * (-2.0 * state[1] + 14.0) / std::pow(std::pow(state[0] + 7.0, 2) + std::pow(state[1] - 7.0, 2) + 1.0, 2);
        H(0, 2) = 0.0;
        H(0, 3) = 0.0;
        H(1, 0) = 1.0 * (-2.0 * state[0] + 14.0) / std::pow(std::pow(state[0] - 7.0, 2) + std::pow(state[1] + 7.0, 2) + 1.0, 2);
        H(1, 1) = 1.0 * (-2.0 * state[1] - 14.0) / std::pow(std::pow(state[0] - 7.0, 2) + std::pow(state[1] + 7.0, 2) + 1.0, 2);
        H(1, 2) = 0.0;
        H(1, 3) = 0.0;
        H(2, 0) = 0.0;
        H(2, 1) = 0.0;
        H(2, 2) = 0.0;
        H(2, 3) = 1.0;
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
    if (observationType_ == "linear") {
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(4, 4);
        matrices.push_back(H);
        matrices.push_back(W);
    } else {
        Eigen::MatrixXd H(3, 4);
        H(0, 0) = 1.0 * (-2 * state[0] - 14.0) / std::pow(std::pow(state[0] + 7.0, 2) + std::pow(state[1] - 7.0, 2) + 1.0, 2);
        H(0, 1) = 1.0 * (-2.0 * state[1] + 14.0) / std::pow(std::pow(state[0] + 7.0, 2) + std::pow(state[1] - 7.0, 2) + 1.0, 2);
        H(0, 2) = 0.0;
        H(0, 3) = 0.0;
        H(1, 0) = 1.0 * (-2.0 * state[0] + 14.0) / std::pow(std::pow(state[0] - 7.0, 2) + std::pow(state[1] + 7.0, 2) + 1.0, 2);
        H(1, 1) = 1.0 * (-2.0 * state[1] - 14.0) / std::pow(std::pow(state[0] - 7.0, 2) + std::pow(state[1] + 7.0, 2) + 1.0, 2);
        H(1, 2) = 0.0;
        H(1, 3) = 0.0;
        H(2, 0) = 0.0;
        H(2, 1) = 0.0;
        H(2, 2) = 0.0;
        H(2, 3) = 1.0;
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(3, 3);
        matrices.push_back(H);
        matrices.push_back(W);
    }

}

void DubinRobot::updateViewer(std::vector<double>& state,
                              std::vector<std::vector<double>>& particles,
                              std::vector<std::vector<double>>& particle_colors)
{
#ifdef USE_OPENRAVE

    /**std::vector<double> dims( {state[0], state[1], 0.025, dim_x_, dim_y_, dim_z_, state[2]});
    std::string name = "dubin";
    viewer_->addBox(name, dims);*/

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
