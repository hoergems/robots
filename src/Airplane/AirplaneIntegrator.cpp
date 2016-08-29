#include <robot_headers/Airplane/AirplaneIntegrator.hpp>

namespace shared
{

AirplaneIntegrator::AirplaneIntegrator():
    cl_(0.0),
    cd_(0.0),
    k_(0.0),
    g_(0.0)
{

}

void AirplaneIntegrator::do_integration(state_type& x,
                                        state_type& control,
                                        state_type& control_error,
                                        state_type& int_times,
                                        state_type& result) const
{
    double t0 = int_times[0];
    double te = int_times[1];
    double step_size = int_times[2];
    tauDest_ = control[0] + control_error[0];
    alphaDest_ = control[1] + control_error[1];
    betaDest_ = control[2] + control_error[2];
    size_t k = integrate_const(adams_bashforth<2, state_type>() ,
                               std::bind(&AirplaneIntegrator::ode , this , pl::_1 , pl::_2 , pl::_3),
                               x , t0 , te , step_size);
    result = x;
}

void AirplaneIntegrator::ode(const state_type& x , state_type& dxdt , double t) const
{
    dxdt.clear();
    dxdt.resize(9);
    dxdt[0] = x[3] * cos(x[7]) * cos(x[6]); // x
    dxdt[1] = x[3] * cos(x[7]) * sin(x[6]); // y
    dxdt[2] = x[3] * sin(x[7]); // z
    dxdt[3] = x[8] * cos(x[5]) - Cd_ * k_ * std::pow(x[3], 2) - g_ * sin(x[7]); // v
    dxdt[4] = alphaDest_ - x[4]; // alpha
    dxdt[5] = betaDest_ - x[5]; // beta
    dxdt[6] = x[3] * (sin(x[4]) / cos(x[7])) * ((x[8] * sin(x[5]) / x[3]) + Cl_ * k_ * x[3]); // theta
    dxdt[7] = cos(x[4]) * ((x[8] * sin(x[5]) / x[3]) + Cl_ * k_ * x[3]) - g_ * (cos(x[7]) / x[3]); // omega
    dxdt[8] = tauDest_ - x[8]; // tau
}

void AirplaneIntegrator::setup(double cl, double cd, double k, double g)
{
    cl_ = cl;
    cd_ = cd;
    k_ = k;
    g_ = g;
}

}
