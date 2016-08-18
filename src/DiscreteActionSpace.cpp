#include "include/DiscreteActionSpace.hpp"

namespace shared
{
DiscreteActionSpace::DiscreteActionSpace():
    shared::ActionSpace()
{

}

std::string DiscreteActionSpace::getType() const {
    std::string type = "discrete";
    return type;
}
}
