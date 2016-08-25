#include "include/auv.hpp"

namespace shared
{
AUV::AUV(std::string robot_file):
    Robot(robot_file),
    dim_x_(0.0),
    dim_y_(0.0),
    dim_z_(0.0)
{
    propagator_ = std::make_shared<shared::AUVPropagator>();
    dim_x_ = 0.5;
    dim_y_ = 0.5;
    dim_z_ = 0.5;
}

void AUV::createRobotCollisionObjects(const std::vector<double>& state,
                                      std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const
{
    double x = state[0];
    double y = state[1];
    fcl::Vec3f trans_vec(x, y, 0.01);
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
    std::shared_ptr<fcl::CollisionObject> coll_obj =
        std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
    collision_objects.push_back(coll_obj);
}

bool AUV::makeActionSpace(bool normalizedActionSpace)
{
    actionSpace_ = std::make_shared<shared::DiscreteActionSpace>(normalizedActionSpace);
    unsigned int numDimensions = 1;
    actionSpace_->setNumDimensions(numDimensions);
    actionSpace_->setActionLimits(lowerControlLimits_, upperControlLimits_);
    static_cast<shared::AUVPropagator*>(propagator_.get())->setActionSpace(actionSpace_);
}

bool AUV::makeObservationSpace(const shared::ObservationSpaceInfo& observationSpaceInfo)
{
    observationSpace_ = std::make_shared<shared::DiscreteObservationSpace>(observationSpaceInfo);
    observationSpace_->setDimension(2);
    std::vector<std::vector<double>> observations;

    // Get the observations using a serializer

    static_cast<shared::DiscreteObservationSpace*>(observationSpace_.get())->addObservations(observations);
    return true;
}

bool AUV::getObservation(std::vector<double>& state, std::vector<double>& observationError, std::vector<double>& observation) const
{
    return getObservation(state, observation);
}

bool AUV::getObservation(std::vector<double>& state, std::vector<double>& observation) const
{
    if (static_cast<shared::DiscreteObservationSpace*>(observationSpace_.get())->observationExists(state)) {
        observation = state;
    } else {
        observation.clear();
        observation.push_back(0);
        observation.push_back(0);
    }
    return true;
}

double AUV::calcLikelihood(std::vector<double>& state, std::vector<double>& observation)
{
    std::vector<double> transformedState;
    transformToObservationSpace(state, transformedState);
    bool isSame = true;
    for (size_t i = 0; i < observation.size(); i++) {
        if (transformedState[i] != observation[i]) {
            isSame = false;
        }
    }

    if (isSame) {
        return 1.0;
    }

    return 0.0;
}

void AUV::transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const
{
    getObservation(state, res);
}

int AUV::getStateSpaceDimension() const
{
    return 2;
}

int AUV::getDOF() const
{
    return 2;
}

void AUV::makeNextStateAfterCollision(std::vector<double>& previous_state,
                                      std::vector<double>& colliding_state,
                                      std::vector<double>& next_state)
{
    next_state = previous_state;
}

bool AUV::isTerminal(std::vector<double>& state) const
{
    double dist = distanceGoal(state);
    if (dist < goal_radius_) {
        return true;
    }

    return false;
}

double AUV::distanceGoal(std::vector<double>& state) const
{
    assert(goal_position_.size() != 0 && "DubinRobot: No goal area set. Cannot calculate distance!");
    double x = state[0];
    double y = state[1];

    double dist = std::pow(goal_position_[0] - x, 2);
    dist += std::pow(goal_position_[1] - y, 2);
    return std::sqrt(dist);
}

void AUV::setGravityConstant(double gravity_constant)
{

}

void AUV::getLinearObservationDynamics(const std::vector<double>& state,
                                       Eigen::MatrixXd& H,
                                       Eigen::MatrixXd& W) const
{

}

void AUV::getLinearProcessMatrices(const std::vector<double>& state,
                                   std::vector<double>& control,
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
    std::vector<double> elem0( { -1.0, 0.0});
    std::vector<double> elem1( {0.0, 0.0});
    std::vector<double> elem2( {1.0, 0.0});
    elements.push_back(std::make_pair(elem0, 0.1));
    elements.push_back(std::make_pair(elem1, 0.8));
    elements.push_back(std::make_pair(elem2, 0.1));
    static_cast<Eigen::WeightedDiscreteDistribution<double> *>(process_distribution_.get())->setElements(elements);
    setStateCovarianceMatrix(covariance_matrix);
}

void AUV::makeObservationDistribution(Eigen::MatrixXd& mean,
                                      Eigen::MatrixXd& covariance_matrix,
                                      unsigned long seed)
{
    observation_distribution_ = std::make_shared<Eigen::WeightedDiscreteDistribution<double>>();
}

void AUV::updateViewer(std::vector<double>& state,
                       std::vector<std::vector<double>>& particles,
                       std::vector<std::vector<double>>& particle_colors)
{
#ifdef USE_OPENRAVE
    std::vector<std::string> names;
    std::vector<std::vector<double>> dims;
    std::vector<std::vector<double>> colors;
    std::string name = "auv";
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
