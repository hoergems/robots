#include <robot_headers/ContinuousObservationSpace.hpp>

namespace shared
{
ContinuousObservationSpace::ContinuousObservationSpace(const ObservationSpaceInfo& observationSpaceInfo):
    ObservationSpace(observationSpaceInfo),
    lowerLimits_(),
    upperLimits_()
{

}

void ContinuousObservationSpace::setLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) {
    lowerLimits_ = lowerLimits;
    upperLimits_ = upperLimits;
}

void ContinuousObservationSpace::getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
    lowerLimits = lowerLimits_;
    upperLimits = upperLimits_;
}

}
