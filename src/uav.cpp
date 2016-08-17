#include "include/uav.hpp"

namespace shared
{
UAV::UAV(std::string robot_file):
    Robot(robot_file)
{

}

void UAV::createRobotCollisionObjects(const std::vector<double>& state,
                                      std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const
{

}

bool UAV::makeObservationSpace(const shared::ObservationSpaceInfo &observationSpaceInfo)
{
    return true;
}

bool UAV::getObservation(std::vector<double>& state, std::vector<double>& observationError, std::vector<double>& observation) const
{
    return true;
}

bool UAV::getObservation(std::vector<double>& state, std::vector<double>& observation)
{
    return true;
}

void UAV::transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const
{
    
}

int UAV::getStateSpaceDimension() const
{
    return 2;
}

int UAV::getControlSpaceDimension() const
{
    return 1;
}

int UAV::getDOF() const
{
    return 2;
}

void UAV::makeNextStateAfterCollision(std::vector<double>& previous_state,
        std::vector<double>& colliding_state,
        std::vector<double>& next_state)
{
    
}

bool UAV::isTerminal(std::vector<double>& state) const
{
    return false;
}

double UAV::distanceGoal(std::vector<double>& state) const
{ 
    
}

void UAV::setGravityConstant(double gravity_constant)
{

}

void UAV::getLinearObservationDynamics(const std::vector<double>& state,
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& W) const
{ 
    
}

void UAV::getLinearProcessMatrices(const std::vector<double>& state,
        std::vector<double>& control,
        double& duration,
        std::vector<Eigen::MatrixXd>& matrices) const
{ 
    
}

void UAV::updateViewer(std::vector<double>& state,
                       std::vector<std::vector<double>>& particles,
                       std::vector<std::vector<double>>& particle_colors)
{
#ifdef USE_OPENRAVE
#endif
}

}
