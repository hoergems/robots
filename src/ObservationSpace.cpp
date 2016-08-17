#include "include/ObservationSpace.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace shared
{

ObservationSpace::ObservationSpace(const shared::ObservationSpaceInfo &observationSpaceInfo):
    dimension_(1),    
    observationSpaceInfo_(observationSpaceInfo)
{
    cout << "made observation space" << endl;
}

void ObservationSpace::setDimension(unsigned int dimension) {
    dimension_ = dimension;
}

unsigned int ObservationSpace::getDimension() const {    
    return dimension_;
}

const shared::ObservationSpaceInfo ObservationSpace::getObservationSpaceInfo() const {
    return observationSpaceInfo_;
}

}
