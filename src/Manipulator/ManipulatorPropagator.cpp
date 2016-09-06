#include <robot_headers/Manipulator/ManipulatorPropagator.hpp>
#include <iostream>

using std::cout;
using std::endl;

namespace frapu
{

ManipulatorPropagator::ManipulatorPropagator():
    frapu::Propagator(),
    integrator_(new Integrate())
{
}


std::shared_ptr<Integrate> ManipulatorPropagator::getIntegrator()
{
    return integrator_;
}

bool ManipulatorPropagator::propagate_linear(const std::vector<double>& current_joint_values,
        const std::vector<double>& control,
        const std::vector<double>& control_error,
        const double duration,
        std::vector<double>& result)
{

    std::vector<double> c;
    bool allZeros = true;

    for (size_t i = 0; i < control.size(); i++) {
        if (control[i] != 0) {
            allZeros = false;
            c.push_back(1.0);
        } else {
            // Add no uncertainty if joint input is 0
            c.push_back(0.0);
        }
    }

    if (allZeros) {
        return false;
    }

    for (size_t i = 0; i < control.size(); i++) {
        result.push_back(current_joint_values[i] +
                         duration * control[i] +
                         c[i] * control_error[i]);
    }
    for (size_t i = 0; i < control.size(); i++) {
        result.push_back(0.0);
    }
    return true;

}

bool ManipulatorPropagator::propagate_nonlinear_first_order(const std::vector<double>& current_joint_values,
        const std::vector<double>& current_joint_velocities,
        std::vector<double>& control,
        std::vector<double>& control_error_vec,
        std::vector<double>& nominal_state,
        std::vector<double>& nominal_control,
        const double simulation_step_size,
        const double duration,
        std::vector<double>& result)
{
    std::vector<double> state;

    for (size_t i = 0; i < current_joint_values.size(); i++) {
        state.push_back(current_joint_values[i]);
    }
    for (size_t i = 0; i < current_joint_values.size(); i++) {
        state.push_back(current_joint_velocities[i]);
    }

    std::vector<double> integration_result;
    std::vector<double> inte_times( {0.0, duration, simulation_step_size});
    integrator_->do_integration_first_order(state,
                                            control,
                                            control_error_vec,
                                            nominal_state,
                                            nominal_control,
                                            inte_times,
                                            integration_result);

    std::vector<double> newJointValues;
    std::vector<double> newJointVelocities;

    for (size_t i = 0; i < integration_result.size() / 2; i++) {
        newJointValues.push_back(integration_result[i]);
    }

    for (size_t i = integration_result.size() / 2; i < integration_result.size(); i++) {
        newJointVelocities.push_back(integration_result[i]);
    }

    for (size_t i = 0; i < newJointValues.size(); i++) {
        result.push_back(newJointValues[i]);
    }

    for (size_t i = 0; i < newJointVelocities.size(); i++) {
        result.push_back(newJointVelocities[i]);
    }

    return true;

}

bool ManipulatorPropagator::propagate_nonlinear_second_order(const std::vector<double>& current_joint_values,
        const std::vector<double>& current_joint_velocities,
        std::vector<double>& control,
        std::vector<double>& control_error_vec,
        std::vector<double>& nominal_state,
        std::vector<double>& nominal_control,
        const double simulation_step_size,
        const double duration,
        std::vector<double>& result)
{
    std::vector<double> state;

    for (size_t i = 0; i < current_joint_values.size(); i++) {
        state.push_back(current_joint_values[i]);
    }
    for (size_t i = 0; i < current_joint_values.size(); i++) {
        state.push_back(current_joint_velocities[i]);
    }

    std::vector<double> integration_result;
    std::vector<double> inte_times( {0.0, duration, simulation_step_size});
    integrator_->do_integration_second_order(state,
            control,
            control_error_vec,
            nominal_state,
            nominal_control,
            inte_times,
            integration_result);

    std::vector<double> newJointValues;
    std::vector<double> newJointVelocities;

    for (size_t i = 0; i < integration_result.size() / 2; i++) {
        newJointValues.push_back(integration_result[i]);
    }

    for (size_t i = integration_result.size() / 2; i < integration_result.size(); i++) {
        newJointVelocities.push_back(integration_result[i]);
    }

    for (size_t i = 0; i < newJointValues.size(); i++) {
        result.push_back(newJointValues[i]);
    }

    for (size_t i = 0; i < newJointVelocities.size(); i++) {
        result.push_back(newJointVelocities[i]);
    }

    return true;

}

bool ManipulatorPropagator::propagateState(const std::vector<double>& currentState,
        const std::vector<double>& control,
        const std::vector<double>& control_error,
        const double& duration,
        const double& simulation_step_size,
        std::vector<double>& result)
{
    std::vector<double> state = currentState;
    std::vector<double> integration_result;
    std::vector<double> inte_times( {0.0, duration, simulation_step_size});
    integrator_->do_integration(state, control, control_error, inte_times, integration_result);
    result = integration_result;
    return true;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_nonlinear_overload, propagateState, 6, 6);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_linear_overload, propagate_linear, 5, 5);

BOOST_PYTHON_MODULE(libpropagator)
{
    using namespace boost::python;

    class_<ManipulatorPropagator>("ManipulatorPropagator", init<>())
    .def("propagateState", &Propagator::propagateState, propagate_nonlinear_overload())
    .def("propagateLinear", &ManipulatorPropagator::propagate_linear, propagate_linear_overload())    
    ;
}


}
