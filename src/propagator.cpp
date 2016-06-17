#include "include/propagator.hpp"

namespace shared {

Propagator::Propagator():
	stateUpperLimits_(),
	stateLowerLimits_(),
	constraintsEnforced_(false){
	
}

void Propagator::setup(std::vector<double> &stateLowerLimits,
		               std::vector<double> &stateUpperLimits,		               
		               bool &enforce_constraints) {
	stateLowerLimits_ = stateLowerLimits;
	stateUpperLimits_ = stateUpperLimits;	
	constraintsEnforced_ = enforce_constraints;
}

bool Propagator::getConstraintsEnforced() {
	return constraintsEnforced_;
}

void Propagator::enforceConstraints(bool enforce) {
	constraintsEnforced_ = enforce;
}


}