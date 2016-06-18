#include "include/DubinPropagator.hpp"

namespace shared {

DubinPropagator::DubinPropagator():
		shared::Propagator(),
		d_()
{
	
}

void DubinPropagator::setD(double &d) {
	d_ = d;
}

bool DubinPropagator::propagateState(const std::vector<double> &currentState,
	            std::vector<double> &control,
	            std::vector<double> &control_error,				             		             
				const double &duration,
				const double &simulation_step_size,
	            std::vector<double> &result) {
	
}

void DubinPropagator::do_integration(std::vector<double> &x,
	    			                 std::vector<double> &control,
	    			                 std::vector<double> &control_error,
	    			                 std::vector<double> &int_times,
	    			                 std::vector<double> &result) const {
	result.clear();
	result.push_back(x[0] + int_times[1] * x[3] * cos(x[2]));
	result.push_back(x[1] + int_times[1] * x[3] * sin(x[2]));
	result.push_back(x[2] + int_times[1] * tan(control[1] + control_error[1]) / d_);
	result.push_back(x[3] + int_times[1] * (control[0] + control_error[0]));
}

}