#include <robot_headers/ActionSpace.hpp>
#include <memory>

using std::cout;
using std::endl;

namespace shared
{
ActionSpace::ActionSpace(bool normalizedActionSpace):
    numDimensions_(),
    lowerActionLimits_(),
    upperActionLimits_(),
    actionNormalizer_(nullptr),
    normalizedActionSpace_(normalizedActionSpace)
{
    if (normalizedActionSpace) {
        actionNormalizer_ = std::unique_ptr<shared::standardNormalize>(new standardNormalize());
        //actionNormalizer_ = std::make_unique<shared::standardNormalize>(lowerActionLimits_, upperActionLimits_);
    } else {
        actionNormalizer_ = std::unique_ptr<shared::nullNormalize>(new nullNormalize());
        //actionNormalizer_ = std::make_unique<shared::nullNormalize>(lowerActionLimits_, upperActionLimits_);
    }

}

void ActionSpace::setNumDimensions(unsigned int& numDimensions)
{
    numDimensions_ = numDimensions;
}

unsigned int ActionSpace::getNumDimensions() const
{
    return numDimensions_;
}

void setActionLimits(std::shared_ptr<shared::ActionLimits> &actionLimits)
{
    actionLimits_ = actionLimits;
    actionNormalizer_->setActionLimits(actionLimits);
}

void ActionSpace::getActionLimits() const
{
    return actionLimits_;
}

void ActionSpace::normalizeAction(std::vector<double>& action,
                                  std::vector<double>& normalizedAction)
{
    actionNormalizer_->operator()(action, normalizedAction);
}

void ActionSpace::denormalizeAction(std::vector<double>& normalizedAction,
                                    std::vector<double>& action)
{
    actionNormalizer_->denormalizeAction(normalizedAction, action);    
}

bool ActionSpace::isNormalized() const
{
    return normalizedActionSpace_;
}

}
