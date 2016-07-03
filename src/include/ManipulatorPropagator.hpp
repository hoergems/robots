#ifndef MANIPULATOR_PROPAGATOR_HPP_
#define MANIPULATOR_PROPAGATOR_HPP_
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include "propagator.hpp"
#include "integrate.hpp"

namespace shared {

   class ManipulatorPropagator: public Propagator {
   public:
	   ManipulatorPropagator();
	   
	   bool propagate_linear(const std::vector<double> &current_joint_values,
               const std::vector<double> &control,
               const std::vector<double> &control_error,				             		             
               const double duration,
               std::vector<double> &result);
	   
	   virtual bool propagateState(const std::vector<double> &currentState,
	                               std::vector<double> &control,
	                               std::vector<double> &control_error,				             		             
								   const double &duration,
								   const double &simulation_step_size,
	                               std::vector<double> &result) override;
	   
	   bool propagate_nonlinear_first_order(const std::vector<double> &current_joint_values,
	   	   		                        const std::vector<double> &current_joint_velocities,
	   	   		                        std::vector<double> &control,
	   	   		                        std::vector<double> &control_error_vec,
										std::vector<double> &nominal_state,
										std::vector<double> &nominal_control,
	   	   		                        const double simulation_step_size,
	   	   		                        const double duration,
	   	   		                        std::vector<double> &result);
	   
	   bool propagate_nonlinear_second_order(const std::vector<double> &current_joint_values,
	   		                        const std::vector<double> &current_joint_velocities,
	   		                        std::vector<double> &control,
	   		                        std::vector<double> &control_error_vec,
									std::vector<double> &nominal_state,
									std::vector<double> &nominal_control,
	   		                        const double simulation_step_size,
	   		                        const double duration,
	   		                        std::vector<double> &result);
	   
	   
	   /**
	    * Gets the underlying integrator
	    */
	   std::shared_ptr<Integrate> getIntegrator();
	   
   private:
	   std::shared_ptr<Integrate> integrator_;
   };

}

#endif
