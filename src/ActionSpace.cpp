#include "include/ActionSpace.hpp"

namespace shared
{
ActionSpace::ActionSpace():
    numDimensions_()
{

}

void ActionSpace::setNumDimensions(unsigned int &numDimensions) {
    numDimensions_ = numDimensions;
}

unsigned int ActionSpace::getNumDimensions() const {
    return numDimensions_;
}

}
