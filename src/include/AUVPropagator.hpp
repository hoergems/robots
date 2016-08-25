#ifndef __AUV_PROPAGATOR_HPP__
#define __AUV_PROPAGATOR_HPP__
#include <memory>
#include <unistd.h>
#include "propagator.hpp"
#include "DiscreteActionSpace.hpp"

namespace shared
{

class AUVPropagator: public Propagator
{
public:
    AUVPropagator();

    virtual bool propagateState(const std::vector<double>& currentState,
                                std::vector<double>& control,
                                std::vector<double>& control_error,
                                const double& duration,
                                const double& simulation_step_size,
                                std::vector<double>& result) override;
				
    void setActionSpace(std::shared_ptr<shared::ActionSpace> &actionSpace);
    
private:
    std::shared_ptr<shared::ActionSpace> actionSpace_;
    
};

}

#endif
