#include "include/DiscreteObservationSpace.hpp"

namespace shared
{
DiscreteObservationSpace::DiscreteObservationSpace(const ObservationSpaceInfo& observationSpaceInfo):
    ObservationSpace(observationSpaceInfo),
    observationMap_()    
{

}

void DiscreteObservationSpace::addObservations(const std::vector<std::vector<double>> &observationStates) {
    std::size_t hashValue;
    for (auto &observationState: observationStates) {
	hashValue = calcHashValue_(observationState);	
	if (observationMap_.count(hashValue) == 0) {
	    observationMap_[hashValue] = observationState;	    
	}
    }
}

void DiscreteObservationSpace::removeObservations(const std::vector<std::vector<double>> &observationStates) {
    std::size_t hashValue;
    for (auto &observationState: observationStates) {
	hashValue = calcHashValue_(observationState);
	observationMap_.erase(hashValue);	
    }
}

bool DiscreteObservationSpace::observationExists(std::vector<double> &observation) const {
    std::size_t hashValue = calcHashValue_(observation);
    if (observationMap_.count(hashValue) > 0) {
	return true;
    }
    
    return false;
}

unsigned int DiscreteObservationSpace::getNumObservations()
{
    return observationMap_.size();
}

void DiscreteObservationSpace::hashCombine_(std::size_t &seed, double const &v) const{
    std::hash<double> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

size_t DiscreteObservationSpace::calcHashValue_(const std::vector<double> &observation) const{
    std::size_t hashValue = 0;
    for (auto &k: observation) {
	hashCombine_(hashValue, k);
    }
    
    return hashValue;    
}

}
