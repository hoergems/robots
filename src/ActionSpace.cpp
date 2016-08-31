#include <robot_headers/ActionSpace.hpp>
#include <memory>

using std::cout;
using std::endl;

namespace frapu
{
ActionSpace::ActionSpace(bool normalizedActionSpace):
    numDimensions_(),    
    actionNormalizer_(nullptr),
    normalizedActionSpace_(normalizedActionSpace)
{
    if (normalizedActionSpace) {
        actionNormalizer_ = std::unique_ptr<standardNormalize>(new standardNormalize());
        //actionNormalizer_ = std::make_unique<shared::standardNormalize>(lowerActionLimits_, upperActionLimits_);
    } else {
        actionNormalizer_ = std::unique_ptr<nullNormalize>(new nullNormalize());
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

void ActionSpace::setActionLimits(frapu::ActionLimitsSharedPtr &actionLimits)
{
    actionLimits_ = actionLimits;
    actionNormalizer_->setActionLimits(actionLimits);
}

ActionLimitsSharedPtr ActionSpace::getActionLimits() const
{
    return actionLimits_;
}

void ActionSpace::normalizeAction(ActionSharedPtr& action)
{
    actionNormalizer_->operator()(action);
}

void ActionSpace::denormalizeAction(ActionSharedPtr& action)
{
    actionNormalizer_->denormalizeAction(action);    
}

bool ActionSpace::isNormalized() const
{
    return normalizedActionSpace_;
}

}
