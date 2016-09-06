#include <robot_headers/Airplane/AirplanePropagator.hpp>

namespace frapu
{
AirplanePropagator::AirplanePropagator():
    integrator_(std::make_shared<AirplaneIntegrator>())
{

}

bool AirplanePropagator::propagateState(const std::vector<double>& currentState,
                                        const std::vector<double>& control,
                                        const std::vector<double>& control_error,
                                        const double& duration,
                                        const double& simulation_step_size,
                                        std::vector<double>& result)
{
    std::vector<double> currentStateNonConst = currentState;
    std::vector<double> intTimes(3);
    intTimes[0] = 0.0;
    intTimes[1] = duration;
    intTimes[2] = simulation_step_size;
    integrator_->do_integration(currentStateNonConst, control, control_error, intTimes, result);
}

std::shared_ptr<AirplaneIntegrator> AirplanePropagator::getIntegrator() const
{
    return integrator_;
}
}
