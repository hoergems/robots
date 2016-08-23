#include "include/AUVPropagator.hpp"

namespace shared
{

AUVPropagator::AUVPropagator():
    shared::Propagator(),
    actionSpace_(nullptr)
{

}

bool AUVPropagator::propagateState(const std::vector<double>& currentState,
                                   std::vector<double>& control,
                                   std::vector<double>& control_error,
                                   const double& duration,
                                   const double& simulation_step_size,
                                   std::vector<double>& result)
{
    
}

void AUVPropagator::setActionSpace(std::shared_ptr<shared::ActionSpace> &actionSpace) {
    actionSpace_ = actionSpace;
}

}