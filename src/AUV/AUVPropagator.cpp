#include <robot_headers/AUV/AUVPropagator.hpp>

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
    result = currentState;
    if (control[0] == 1.0) {
        result[1] += 0.01;
    } else if (control[0] == 2.0) {
        result[0] += 0.01;
        result[1] += 0.01;
    } else if (control[0] == 3.0) {
        result[0] += 0.01;
    } else if (control[0] == 4.0) {
        result[0] += 0.01;
        result[1] -= 0.01;
    } else if (control[0] == 5.0) {
        result[1] -= 0.01;
    }    
   
    result[0] += control_error[0];    
}

void AUVPropagator::setActionSpace(std::shared_ptr<shared::ActionSpace>& actionSpace)
{
    actionSpace_ = actionSpace;
}

}
