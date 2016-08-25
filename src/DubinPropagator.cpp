#include "include/DubinPropagator.hpp"

namespace shared
{

using std::cout;
using std::endl;

DubinPropagator::DubinPropagator():
    shared::Propagator(),
    d_()
{

}

void DubinPropagator::setD(double& d)
{
    d_ = d;
}

bool DubinPropagator::propagateState(const std::vector<double>& currentState,
                                     std::vector<double>& control,
                                     std::vector<double>& control_error,
                                     const double& duration,
                                     const double& simulation_step_size,
                                     std::vector<double>& result)
{
    result.clear();
    result.push_back(currentState[0] + duration * currentState[3] * cos(currentState[2]));
    result.push_back(currentState[1] + duration * currentState[3] * sin(currentState[2]));
    result.push_back(currentState[2] + duration * currentState[3] * tan(control[1] + control_error[1]) / d_);
    result.push_back(currentState[3] + duration * (control[0] + control_error[0]));
}

}
