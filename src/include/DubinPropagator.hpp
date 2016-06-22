#ifndef DUBIN_PROPAGATOR_HPP_
#define DUBIN_PROPAGATOR_HPP_
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include "propagator.hpp"

namespace shared {

typedef std::vector<double> state_type;

class DubinPropagator: public Propagator {
public:
	DubinPropagator();
	
	virtual bool propagateState(const std::vector<double> &currentState,
	            std::vector<double> &control,
	            std::vector<double> &control_error,				             		             
				const double &duration,
				const double &simulation_step_size,
	            std::vector<double> &result) override;
	
	void setD(double &d); 
	
private:	
	double d_;
	
};


}

#endif

