#ifndef __DISCRETE_OBSERVATION_SPACE__
#define __DISCRETE_OBSERVATION_SPACE__
#include "ObservationSpace.hpp"
#include <map>


namespace shared
{
class DiscreteObservationSpace: public ObservationSpace
{
public:
    DiscreteObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);
    
    /**
     * Adds a vector of observations to the set of observations
     */
    void addObservations(const std::vector<std::vector<double>> &observationStates);
    
    /**
     * Removes a vector of observations to the set of observations
     */
    void removeObservations(const std::vector<std::vector<double>> &observationStates);
    
    /**
     * Check if a particular observation is contained within the set of discrete observations
     */
    bool observationExists(std::vector<double> &observation) const;
    
    /**
     * Get the number of observations
     */
    unsigned int getNumObservations();
    
private:
    
    /**
     * A map for fast observation lookup
     */
    std::map<std::size_t, std::vector<double>> observationMap_;
    
    /**
     * Calculate the hash value for a particular observation
     */
    size_t calcHashValue_(const std::vector<double> &observation) const;

};
}

#endif
