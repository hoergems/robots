#ifndef _DUBIN_ROBOT_HPP_
#define _DUBIN_ROBOT_HPP_
#include <string>
#include <iostream>
#include <assert.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include "fcl/BV/BV.h" 
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <tinyxml.h>
#include <rbdl_interface/rbdl_interface.hpp>
#include "robot.hpp"
#include "DubinPropagator.hpp"

using std::cout;
using std::endl;

namespace shared {

class DubinRobot: public Robot {
public:
	DubinRobot(std::string robot_file);
	
	void createRobotCollisionObjects(const std::vector<double> &state, 
		       std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const override;
	
	int getStateSpaceDimension() override;
	
	int getControlSpaceDimension() override;
	
	int getDOF() const override;
	
	void getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const override;
	
	void getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const override;
	
	void getLinearProcessMatrices(std::vector<double> &state, 
	    	    			      std::vector<double> &control, 
	    	    			      double &duration,
	    	    			      std::vector<Eigen::MatrixXd> &matrices) const override;
	
	
};

}

#endif
