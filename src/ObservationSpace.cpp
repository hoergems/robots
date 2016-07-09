#include "include/ObservationSpace.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace shared
{

ObservationSpace::ObservationSpace(std::string &observationType):
    dimension_(1),
    lowerLimits_(),
    upperLimits_(),
    observationType_(observationType)
{
    cout << "made observation space" << endl;
}

void ObservationSpace::setDimension(unsigned int dimension) {
    dimension_ = dimension;
}

void ObservationSpace::setLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) {
    lowerLimits_ = lowerLimits;
    upperLimits_ = upperLimits;
}

void ObservationSpace::getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
    lowerLimits = lowerLimits_;
    upperLimits = upperLimits_;
}

unsigned int ObservationSpace::getDimension() const {    
    return dimension_;
}

std::string ObservationSpace::getObservationType() const {
    return observationType_;
}

}
