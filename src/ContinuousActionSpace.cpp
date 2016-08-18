#include "include/ContinuousActionSpace.hpp"

namespace shared
{
ContinuousActionSpace::ContinuousActionSpace():
    shared::ActionSpace()
{

}

std::string ContinuousActionSpace::getType() const {
    std::string type = "continuous";
    return type;
}
}
