#include "include/ActionSpace.hpp"

namespace shared
{
ActionSpace::ActionSpace():
    numDimensions_(),
    lowerActionLimits_(),
    upperActionLimits_()
{

}

void ActionSpace::setNumDimensions(unsigned int& numDimensions)
{
    numDimensions_ = numDimensions;
}

unsigned int ActionSpace::getNumDimensions() const
{
    return numDimensions_;
}

void ActionSpace::setActionLimits(std::vector<double>& lowerActionLimits,
                                  std::vector<double>& upperActionLimits)
{
    lowerActionLimits_ = lowerActionLimits;
    upperActionLimits_ = upperActionLimits;
}

void ActionSpace::normalizeAction(std::vector<double>& action,
                                  std::vector<double>& normalizedAction)
{
    normalizedAction.clear();
    normalizedAction.resize(action.size());
    for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
        normalizedAction[i] = (action[i] - lowerActionLimits_[i]) / (upperActionLimits_[i] - lowerActionLimits_[i]);
    }
}

void ActionSpace::denormalizeAction(std::vector<double>& normalizedAction,
                                    std::vector<double>& action)
{
    action.clear();
    action.resize(action.size());
    for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
        action[i] = normalizedAction[i] * (upperActionLimits_[i] - lowerActionLimits_[i]) + lowerActionLimits_[i];
    }
}

}
