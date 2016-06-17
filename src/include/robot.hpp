#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_
#include <boost/python.hpp>
#include "fcl/BV/BV.h" 
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "propagator.hpp"

#ifdef USE_URDF
   #include <viewer_interface/viewer_interface.hpp>
#endif

namespace shared {

class Robot {
public:
	Robot(std::string robot_file);
	
	virtual ~Robot() = default;
	
	bool propagateState(std::vector<double> &current_state,
			            std::vector<double> &control_input,
			            std::vector<double> &control_error,
			            double duration,
			            double simulation_step_size,			            
			            std::vector<double> &result);
	
	virtual void createRobotCollisionObjects(const std::vector<double> &state, 
	    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const = 0;
	
	virtual int getStateSpaceDimension() = 0;
	
	virtual int getControlSpaceDimension() = 0;
	
	virtual int getDOF() = 0;
	
	virtual void enforceConstraints(bool enforce);
	
	virtual bool constraintsEnforced();
	
	virtual void getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const = 0;
	
	virtual void getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const = 0;
	
	virtual bool checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const;
	
	virtual bool checkSelfCollision(const std::vector<double> &state) const;
	
#ifdef USE_URDF
	void setupViewer(std::string model_file, std::string environment_file);
	
	void setParticlePlotLimit(unsigned int particle_plot_limit);
#endif

protected:
	bool constraints_enforced_;
	
	std::string robot_file_;
	
	std::shared_ptr<shared::Propagator> propagator_;

#ifdef USE_URDF
    std::shared_ptr<shared::ViewerInterface> viewer_;
#else
    std::shared_ptr<double> viewer_;
#endif
    
};

struct RobotWrapper: Robot, boost::python::wrapper<Robot> {
public:
	RobotWrapper(std::string robot_file):
		Robot(robot_file)
	{
		
	}
	
	int getDOF() {
		return this->get_override("getDOF")();
	}
	
	int getStateSpaceDimension() {
		return this->get_override("getStateSpaceDimension")();		
	}
	
	int getControlSpaceDimension() {
		return this->get_override("getControlSpaceDimension")();
	}
	
	void createRobotCollisionObjects(const std::vector<double> &state, 
		    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const {
		this->get_override("createRobotCollisionObjects")(state, collision_objects);
	}
	
	void getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
		this->get_override("getStateLimits")(lowerLimits, upperLimits);
	}
	
	void getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
		this->get_override("getControlLimits")(lowerLimits, upperLimits);
	}
	
};

}

#endif