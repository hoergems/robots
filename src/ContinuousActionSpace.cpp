#include <robot_headers/ActionSpace.hpp>

namespace frapu
{
ContinuousActionSpace::ContinuousActionSpace(const ActionSpaceInfo &actionSpaceInfo):
    ActionSpace(actionSpaceInfo)
{

}

std::string ContinuousActionSpace::getType() const {
    std::string type = "continuous";
    return type;
}
}
