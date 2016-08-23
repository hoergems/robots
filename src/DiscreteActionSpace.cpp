#include "include/DiscreteActionSpace.hpp"

namespace shared
{
DiscreteActionSpace::DiscreteActionSpace(bool normalizedActionSpace):
    shared::ActionSpace(normalizedActionSpace)
{

}

std::string DiscreteActionSpace::getType() const {
    std::string type = "discrete";
    return type;
}
}
