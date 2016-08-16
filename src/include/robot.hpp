#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "propagator.hpp"
#include <random>
#include "MultNormal.hpp"
#include "ObservationSpace.hpp"

#ifdef USE_OPENRAVE
#include <viewer_interface/viewer_interface.hpp>
#endif

using std::cout;
using std::endl;

namespace shared
{

class Robot
{
public:
    Robot(std::string robot_file);

    virtual ~Robot() = default;

    bool propagateState(const std::vector<double>& current_state,
                        std::vector<double>& control_input,
                        std::vector<double>& control_error,
                        double duration,
                        double simulation_step_size,
                        std::vector<double>& result);

    virtual bool makeObservationSpace(std::string& observationType) = 0;

    virtual bool getObservation(std::vector<double>& state, std::vector<double>& observation) = 0;
    
    virtual bool getObservation(std::vector<double> &state, std::vector<double> &observationError, std::vector<double>& observation) const = 0;

    virtual void createRobotCollisionObjects(const std::vector<double>& state,
            std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const = 0;

    virtual int getStateSpaceDimension() const = 0;

    virtual int getControlSpaceDimension() const = 0;

    virtual int getDOF() const = 0;

    virtual void makeNextStateAfterCollision(std::vector<double>& previous_state,
            std::vector<double>& colliding_state,
            std::vector<double>& next_state) = 0;

    virtual void setGoalArea(std::vector<double>& goal_position, double& goal_radius);

    virtual void setGravityConstant(double gravity_constant) = 0;

    virtual void setNewtonModel();

    virtual void enforceConstraints(bool enforce);

    virtual bool constraintsEnforced();

    virtual bool enforceConstraints(std::vector<double>& state) const;

    virtual bool enforceControlConstraints(std::vector<double>& control) const;

    virtual void getStateLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const;

    virtual void getControlLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const;

    virtual void sampleRandomControl(std::vector<double>& control, std::default_random_engine* randGen, std::string &actionSamplingStrategy);

    virtual void getLinearProcessMatrices(const std::vector<double>& state,
                                          std::vector<double>& control,
                                          double& duration,
                                          std::vector<Eigen::MatrixXd>& matrices) const = 0;
    virtual void getLinearObservationDynamics(const std::vector<double>& state,
            Eigen::MatrixXd& H,
	    Eigen::MatrixXd& W) const = 0;

    virtual bool isTerminal(std::vector<double>& state) const = 0;

    virtual double distanceGoal(std::vector<double>& state) const = 0;

    virtual bool checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const;

    virtual bool checkSelfCollision(const std::vector<double>& state) const;

    virtual void setStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix);

    virtual void getStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) const;

    virtual void setObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix);

    virtual void getObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) const;

    void setProcessDistribution(std::shared_ptr<Eigen::EigenMultivariateNormal<double>>& distribution);

    void setObservationDistribution(std::shared_ptr<Eigen::EigenMultivariateNormal<double>>& distribution);

    void setObservationType(std::string observationType);

    shared::ObservationSpace* getObservationSpace() const;

    virtual void transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const = 0;
    
    /*** Methods for viewer interface ***/
    virtual void setupViewer(std::string model_file, std::string environment_file);
    
    virtual void resetViewer(std::string model_file, std::string environment_file);
    
    virtual void updateViewer(std::vector<double>& state,
                              std::vector<std::vector<double>>& particles,
                              std::vector<std::vector<double>>& particle_colors) = 0;
    

    void getCameraImage(std::vector<uint8_t>& image, int width, int height);

    virtual void setParticlePlotLimit(unsigned int particle_plot_limit);

    virtual void addBox(std::string name, std::vector<double> dims);
    
    virtual void removeBox(std::string name);

protected:
    bool constraints_enforced_;

    std::string robot_file_;

    std::shared_ptr<shared::Propagator> propagator_;

    Eigen::MatrixXd state_covariance_matrix_;

    Eigen::MatrixXd observation_covariance_matrix_;

    std::vector<double> goal_position_;

    double goal_radius_;

    std::vector<double> lowerStateLimits_;

    std::vector<double> upperStateLimits_;

    std::vector<double> lowerControlLimits_;

    std::vector<double> upperControlLimits_;

    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> process_distribution_;

    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> observation_distribution_;

    std::string observationType_;

    std::shared_ptr<shared::ObservationSpace> observationSpace_;

#ifdef USE_OPENRAVE
    std::shared_ptr<shared::ViewerInterface> viewer_;
#else
    std::shared_ptr<double> viewer_;
#endif

};

struct RobotWrapper: Robot, boost::python::wrapper<Robot> {
public:
    RobotWrapper(std::string robot_file):
        Robot(robot_file) {

    }

    int getDOF() const {
        return this->get_override("getDOF")();
    }

    int getStateSpaceDimension() const {
        return this->get_override("getStateSpaceDimension")();
    }

    int getControlSpaceDimension() const {
        return this->get_override("getControlSpaceDimension")();
    }

    void addBox(std::string name, std::vector<double> dims) {
        this->get_override("addBox")(name, dims);
    }
    
    void removeBox(std::string name) {
	this->get_override("removeBox")(name);
    }

    void createRobotCollisionObjects(const std::vector<double>& state,
                                     std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const {
        this->get_override("createRobotCollisionObjects")(state, collision_objects);
    }

    void getStateLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const {
        this->get_override("getStateLimits")(lowerLimits, upperLimits);
    }

    void getControlLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const {
        this->get_override("getControlLimits")(lowerLimits, upperLimits);
    }

    void getLinearProcessMatrices(const std::vector<double>& state,
                                  std::vector<double>& control,
                                  double& duration,
                                  std::vector<Eigen::MatrixXd>& matrices) const {
        this->get_override("getLinearProcessMatrices")(state, control, duration, matrices);
    }

    void setStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) {
        this->get_override("setStateCovarianceMatrix")(state_covariance_matrix);
    }

    void getStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) const {
        this->get_override("getStateCovarianceMatrix")(state_covariance_matrix);
    }

    void setObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) {
        this->get_override("setStateCovarianceMatrix")(observation_covariance_matrix);
    }

    void getObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) const {
        this->get_override("getObservationCovarianceMatrix")(observation_covariance_matrix);
    }

    bool getObservation(std::vector<double>& state, std::vector<double>& observation) {
        this->get_override("getObservation")(state, observation);
    }

    bool isTerminal(std::vector<double>& state) const {
        return this->get_override("isTerminal")(state);
    }

    double distanceGoal(std::vector<double>& state) const {
        return this->get_override("distanceGoal")(state);
    }

    bool enforceConstraints(std::vector<double>& state) const {
        return this->get_override("enforceConstraints")(state);
    }

    void enforceConstraints(bool enforce) {
        this->get_override("enforceConstraints")(enforce);
    }

    void updateViewer(std::vector<double>& state,
                      std::vector<std::vector<double>>& particles,
                      std::vector<std::vector<double>>& particle_colors) {
        this->get_override("updateViewer")(state, particles, particle_colors);
    }

    void makeNextStateAfterCollision(std::vector<double>& previous_state,
                                     std::vector<double>& colliding_state,
                                     std::vector<double>& next_state) {
        this->get_override("makeNextStateAfterCollision")(previous_state, colliding_state, next_state);
    }

    void sampleRandomControl(std::vector<double>& control, std::default_random_engine* randGen) {
        this->get_override("sampleRandomControl")(control, randGen);
    }

    void setGravityConstant(double gravity_constant) {
        this->get_override("setGravityConstant")(gravity_constant);
    }

    void setNewtonModel() {
        this->get_override("setNewtonModel")();
    }

    void setProcessDistribution(std::shared_ptr<Eigen::EigenMultivariateNormal<double>>& distribution) {
        this->get_override("setProcessDistribution")(distribution);
    }

    void setObservationDistribution(std::shared_ptr<Eigen::EigenMultivariateNormal<double>>& distribution) {
        this->get_override("setObservationDistribution")(distribution);
    }

    void setObservationType(std::string observationType) {
        this->get_override("setObservationType")(observationType);
    }

    bool makeObservationSpace(std::string& observationType) {
        this->get_override("makeObservationSpace")(observationType);
    }

    void transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const{
        this->get_override("transformToObservationSpace")(state, res);
    }

    void getLinearObservationDynamics(const std::vector<double>& state,
                                      Eigen::MatrixXd& H,
				      Eigen::MatrixXd& W) const {
	this->get_override("getLinearObservationDynamics")(state, H, W);
    }
    
    bool getObservation(std::vector<double> &state, std::vector<double> &observationError, std::vector<double>& observation) const {
	this->get_override("getObservation")(state, observationError, observation);
    }
    
    void getCameraImage(std::vector<uint8_t>& image, int width, int height) {
	this->get_override("getCameraImage")(image, width, height);
    }

};

}

#endif
