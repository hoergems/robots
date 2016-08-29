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
    }
    else {
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

void ActionSpace::setActionLimits(std::vector<double>& lowerActionLimits,
                                  std::vector<double>& upperActionLimits)
{
    lowerActionLimits_ = lowerActionLimits;
    upperActionLimits_ = upperActionLimits;
    actionNormalizer_->setActionLimits(lowerActionLimits, upperActionLimits);
}

void ActionSpace::normalizeAction(std::vector<double>& action,
                                  std::vector<double>& normalizedAction)
{
    actionNormalizer_->operator()(action, normalizedAction);    
    /**normalizedAction.clear();
    normalizedAction.resize(action.size());
    for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
        normalizedAction[i] = (action[i] - lowerActionLimits_[i]) / (upperActionLimits_[i] - lowerActionLimits_[i]);
    }*/
}

void ActionSpace::denormalizeAction(std::vector<double>& normalizedAction,
                                    std::vector<double>& action)
{
    actionNormalizer_->denormalizeAction(normalizedAction, action);
    /**action.clear();
    action.resize(normalizedAction.size());    
    for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
        action[i] = normalizedAction[i] * (upperActionLimits_[i] - lowerActionLimits_[i]) + lowerActionLimits_[i];	
    }*/
}

bool ActionSpace::isNormalized() const {
    return normalizedActionSpace_;
}

}
