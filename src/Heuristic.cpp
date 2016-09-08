#include <robot_headers/Heuristic.hpp>
#include <math.h>

namespace frapu
{
Heuristic::Heuristic()
{

}

RRTHeuristic::RRTHeuristic(frapu::PathPlannerSharedPtr& pathPlanner,
                           frapu::CollisionCheckerSharedPtr& collisionChecker,
                           frapu::EnvironmentInfoSharedPtr& environmentInfo,
                           std::function<bool(frapu::RobotStateSharedPtr)> terminalFunction):
    Heuristic(),
    pathPlanner_(pathPlanner),
    collisionChecker_(collisionChecker),
    environmentInfo_(environmentInfo),
    terminalFunction_(terminalFunction)
{

}

double RRTHeuristic::operator()(frapu::HeuristicInfoSharedPtr& heuristicInfo) const
{
    if (!heuristicInfo->rewardModel) {
	frapu::WARNING("Reward model is nullptr. Did you forget to set a reward model in your HeuristicInfo? Returning 0.0");        
        return 0.0;
    }

    frapu::RewardModelSharedPtr rewardModel = heuristicInfo->rewardModel;
    frapu::TrajectorySharedPtr trajectory =
        pathPlanner_->solve(heuristicInfo->currentState, heuristicInfo->timeout);
    if (!trajectory) {
        return 0.0;
    }

    frapu::CollisionReportSharedPtr collisionReport = std::make_shared<frapu::CollisionReport>();
    collisionReport->continuousCollisionCheck = false;
    collisionReport->state1 = nullptr;
    collisionReport->ignoreUnobservableObstacles = true;
    double reward = 0.0;
    for (size_t i = 0; i < trajectory->stateTrajectory.size(); i++) {
        if (terminalFunction_(trajectory->stateTrajectory[i])) {
            reward += std::pow(rewardModel->discountFactor, heuristicInfo->currentStep + i) * rewardModel->exitReward;
            break;
        }

        collisionReport->state2 = trajectory->stateTrajectory[i];
        collisionChecker_->makeCollisionReport(collisionReport);
        if (collisionReport->collides) {
            if (collisionReport->obstacleTraversable) {
                frapu::ObstacleSharedPtr obstacle =
                    environmentInfo_->scene->getObstacle(collisionReport->collidingObstacle);
                double traversalCost = obstacle->getTerrain()->getTraversalCost();
                reward += reward += std::pow(rewardModel->discountFactor, heuristicInfo->currentStep + i) * traversalCost;
            } else {
		frapu::ERROR("WE collide!");
                reward -= std::pow(rewardModel->discountFactor, heuristicInfo->currentStep + i) * rewardModel->illegalActionPenalty;
            }
        }
    }

    return reward;
}

}
