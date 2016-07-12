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

namespace shared
{

struct Beacon {
public:
    Beacon():
        x_(0.0),
        y_(0.0) {

    }

    Beacon(double x, double y):
        x_(x),
        y_(y) {

    }

    double x_;
    double y_;
};

class DubinRobot: public Robot
{
public:
    DubinRobot(std::string robot_file);

    void createRobotCollisionObjects(const std::vector<double>& state,
                                     std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const override;

    int getStateSpaceDimension() const override;

    int getControlSpaceDimension() const override;

    int getDOF() const override;

    bool isTerminal(std::vector<double>& state) const override;

    double distanceGoal(std::vector<double>& state) const override;

    bool getObservation(std::vector<double>& state, std::vector<double>& observation) override;
    
    bool getObservation(std::vector<double> &state, std::vector<double> &observationError, std::vector<double>& observation) const override;

    void transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const override;

    bool makeObservationSpace(std::string& observationType) override;

    void getLinearProcessMatrices(const std::vector<double>& state,
                                  std::vector<double>& control,
                                  double& duration,
                                  std::vector<Eigen::MatrixXd>& matrices) const override;

    void getLinearObservationDynamics(const std::vector<double>& state,
                                      Eigen::MatrixXd& H,
                                      Eigen::MatrixXd& W) const override;

    void makeNextStateAfterCollision(std::vector<double>& previous_state,
                                     std::vector<double>& colliding_state,
                                     std::vector<double>& next_state) override;


    void updateViewer(std::vector<double>& state,
                      std::vector<std::vector<double>>& particles,
                      std::vector<std::vector<double>>& particle_colors) override;

    void setGravityConstant(double gravity_constant) override;

private:
    
    double dim_x_;
    double dim_y_;
    double dim_z_;
    std::vector<shared::Beacon> beacons_;
    double d_;
    
    void getLinearObservationMatrix(const std::vector<double>& state, Eigen::MatrixXd &H) const;

};

}

#endif
