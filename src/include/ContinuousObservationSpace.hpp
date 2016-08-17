#ifndef __CONTINUOUS_OBSERVATION_SPACE__
#define __CONTINUOUS_OBSERVATION_SPACE__
#include "ObservationSpace.hpp"

namespace shared
{
class ContinuousObservationSpace: public ObservationSpace
{
public:
    ContinuousObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);
    
    void setLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits);
    
    void getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const;
    
private:
    std::vector<double> lowerLimits_;
    
    std::vector<double> upperLimits_;

};
}

#endif
