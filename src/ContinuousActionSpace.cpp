#include "include/ContinuousActionSpace.hpp"

namespace shared
{
ContinuousActionSpace::ContinuousActionSpace(bool normalizedActionSpace):
    shared::ActionSpace(normalizedActionSpace)
{

}

std::string ContinuousActionSpace::getType() const {
    std::string type = "continuous";
    return type;
}
}
