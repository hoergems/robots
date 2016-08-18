#ifndef __DISCRETE_ACTION_SPACE_HPP__
#define __DISCRETE_ACTION_SPACE_HPP__
#include "ActionSpace.hpp"

namespace shared
{
class DiscreteActionSpace: public shared::ActionSpace
{
public:
    DiscreteActionSpace();
    
    virtual std::string getType() const override;

};
}

#endif
