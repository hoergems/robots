#include "include/auv.hpp"

namespace shared
{
AUV::AUV(std::string robot_file):
    Robot(robot_file)
{
    propagator_ = std::make_shared<shared::AUVPropagator>();
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

bool AUV::makeActionSpace()
{
    actionSpace_ = std::make_shared<shared::DiscreteActionSpace>();
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
    }
    return true;
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
    return false;
}

double AUV::distanceGoal(std::vector<double>& state) const
{

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

}

void AUV::makeObservationDistribution(Eigen::MatrixXd& mean,
                                      Eigen::MatrixXd& covariance_matrix,
                                      unsigned long seed)
{

}

void AUV::updateViewer(std::vector<double>& state,
                       std::vector<std::vector<double>>& particles,
                       std::vector<std::vector<double>>& particle_colors)
{
#ifdef USE_OPENRAVE
#endif
}

}
