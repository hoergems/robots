#ifndef __INTEGRATOR_HPP__
#define __INTEGRATOR_HPP__
#include <vector>

namespace shared
{

typedef std::vector<double> state_type;
class Integrator
{
public:
    Integrator() {}

    virtual void do_integration(state_type& x,
                                state_type& control,
                                state_type& control_error,
                                state_type& int_times,
                                state_type& result) const = 0;

    virtual void ode(const state_type& x , state_type& dxdt , double t) const = 0;

};
}

#endif
