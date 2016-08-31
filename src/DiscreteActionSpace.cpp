#include <robot_headers/DiscreteActionSpace.hpp>

namespace frapu
{
DiscreteActionSpace::DiscreteActionSpace(bool normalizedActionSpace):
    ActionSpace(normalizedActionSpace)
{

}

std::string DiscreteActionSpace::getType() const {
    std::string type = "discrete";
    return type;
}
}
