#include <robot_headers/ObservationSpace.hpp>
#include <iostream>

using std::cout;
using std::endl;

namespace frapu
{

ObservationSpace::ObservationSpace(const ObservationSpaceInfo &observationSpaceInfo):
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

const ObservationSpaceInfo ObservationSpace::getObservationSpaceInfo() const {
    return observationSpaceInfo_;
}

}
