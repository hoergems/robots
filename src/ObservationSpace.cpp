#include "include/ObservationSpace.hpp"

namespace shared
{

ObservationSpace::ObservationSpace():
    dimension_(1),
    lowerLimits_(),
    upperLimits_()
{
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

}
