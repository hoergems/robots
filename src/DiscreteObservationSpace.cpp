#include "include/DiscreteObservationSpace.hpp"

namespace shared
{
DiscreteObservationSpace::DiscreteObservationSpace(const ObservationSpaceInfo& observationSpaceInfo):
    ObservationSpace(observationSpaceInfo)
{

}

unsigned int DiscreteObservationSpace::getNumObservations()
{
    return 0;
}
}
