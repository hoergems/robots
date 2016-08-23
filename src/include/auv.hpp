#ifndef _AUV_ROBOT_
#define _AUV_ROBOT_
#include <string>
#include <iostream>
#include <assert.h>
#include "robot.hpp"
#include "AUVPropagator.hpp"

namespace shared
{
class AUV: public Robot
{
public:
    AUV(std::string robot_file);

    void createRobotCollisionObjects(const std::vector<double>& state,
                                     std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const override;

    int getStateSpaceDimension() const override;

    int getDOF() const override;

    bool isTerminal(std::vector<double>& state) const override;

    double distanceGoal(std::vector<double>& state) const override;

    bool getObservation(std::vector<double>& state, std::vector<double>& observation) const override;

    bool getObservation(std::vector<double>& state, std::vector<double>& observationError, std::vector<double>& observation) const override;

    void transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const override;

    bool makeActionSpace(bool normalizedActionSpace) override;

    bool makeObservationSpace(const shared::ObservationSpaceInfo& observationSpaceInfo) override;

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

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix,
                                         unsigned long seed) override;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix,
            unsigned long seed) override;

};
}

#endif
