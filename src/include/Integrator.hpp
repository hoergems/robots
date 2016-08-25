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
    
    virtual void ode(const state_type& x , state_type& dxdt , double t) const = 0;

};
}

#endif
