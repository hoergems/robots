#include <robot_headers/ActionSpace.hpp>

namespace frapu
{
DiscreteActionSpace::DiscreteActionSpace(const ActionSpaceInfo &actionSpaceInfo):
    ActionSpace(actionSpaceInfo)
{

}

std::string DiscreteActionSpace::getType() const {
    std::string type = "discrete";
    return type;
}
}
