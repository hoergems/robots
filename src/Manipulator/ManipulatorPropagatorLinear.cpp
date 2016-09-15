#include <robot_headers/Manipulator/ManipulatorPropagatorLinear.hpp>

namespace frapu
{
ManipulatorPropagatorLinear::ManipulatorPropagatorLinear():
    frapu::Propagator()
{
}

bool ManipulatorPropagatorLinear::propagateState(const std::vector<double>& currentState,
        const std::vector<double>& control,
        const std::vector<double>& control_error,
        const double& duration,
        const double& simulation_step_size,
        std::vector<double>& result)
{
    result = std::vector<double>(currentState.size());
    for (size_t i = 0; i < currentState.size(); i++) {
        result[i] = currentState[i] + control[i] + control_error[i];
    }

}

}
