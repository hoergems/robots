#ifndef __CONTINUOUS_ACTION_SPACE_HPP__
#define __CONTINUOUS_ACTION_SPACE_HPP__
#include "ActionSpace.hpp"

namespace shared
{
class ContinuousActionSpace: public shared::ActionSpace
{
public:
    ContinuousActionSpace();
    
    virtual std::string getType() const override;

};
}

#endif