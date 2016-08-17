#ifndef __DISCRETE_OBSERVATION_SPACE__
#define __DISCRETE_OBSERVATION_SPACE__
#include "ObservationSpace.hpp"

namespace shared
{
class DiscreteObservationSpace: public ObservationSpace
{
public:
    DiscreteObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);
    
    unsigned int getNumObservations();

};
}

#endif
