#include <robot_headers/ContinuousActionSpace.hpp>

namespace frapu
{
ContinuousActionSpace::ContinuousActionSpace(bool normalizedActionSpace):
    ActionSpace(normalizedActionSpace)
{

}

std::string ContinuousActionSpace::getType() const {
    std::string type = "continuous";
    return type;
}
}
