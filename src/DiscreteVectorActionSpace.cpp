#include <robot_headers/ActionSpace.hpp>

namespace frapu
{
DiscreteVectorActionSpace::DiscreteVectorActionSpace(const ActionSpaceInfo& actionSpaceInfo):
    DiscreteActionSpace(actionSpaceInfo),
    actionSpaceInfo_(actionSpaceInfo),
    allActionsOrdered_()
{

}

void DiscreteVectorActionSpace::setActionLimits(frapu::ActionLimitsSharedPtr& actionLimits)
{
    if (getNumDimensions() == 0) {
        frapu::ERROR("DiscreteActionSpace: setActionLimits(): number of dimensions not set. Can't set action limits");
    }
    ActionSpace::setActionLimits(actionLimits);
    makeAllActionsInOrder(actionSpaceInfo_.numStepsPerDimension);
}

void DiscreteVectorActionSpace::makeAllActionsInOrder(const unsigned int& numStepsPerDimension)
{
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    unsigned int numDimensions = getNumDimensions();
    ActionLimitsSharedPtr actionLimits = getActionLimits();
    if (!actionLimits) {
        frapu::ERROR("action limits is null");
    }
    
    allActionsOrdered_ = std::vector<frapu::ActionSharedPtr>(std::pow(numStepsPerDimension, numDimensions));

    static_cast<VectorActionLimits*>(actionLimits.get())->getRawLimits(lowerLimits, upperLimits);
    for (long code = 0; code < std::pow(numStepsPerDimension, numDimensions); code++) {
        std::vector<double> ks;
        std::vector<double> ss;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            ks.push_back((upperLimits[i] - lowerLimits[i]) / (numStepsPerDimension - 1));
        }

        double j = code;
        double j_old = code;
        double s = 0;
        for (size_t i = lowerLimits.size() - 1; i != (size_t) - 0; i--) {
            double s;
            j = j_old / std::pow(numStepsPerDimension, i);
            modf(j, &s);
            ss.push_back(s);
            if (i != 1) {
                j = (int)(j_old) % (int)std::pow(numStepsPerDimension, i);
                j_old = j;
            }
        }

        ss.push_back((int)j_old % numStepsPerDimension);
        std::vector<double> actionValues;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            actionValues.push_back(lowerLimits[i] + ss[i] * ks[i]);
        }

        frapu::ActionSharedPtr action(new frapu::DiscreteVectorAction(actionValues));
	static_cast<frapu::DiscreteVectorAction*>(action.get())->setBinNumber(code);
        allActionsOrdered_[code] = action;        
    }
}

ActionSharedPtr DiscreteVectorActionSpace::sampleUniform(std::default_random_engine* randGen) const
{
    unsigned int randNum = std::uniform_int_distribution<unsigned int>(0, allActionsOrdered_.size() - 1)(*randGen);
    return allActionsOrdered_[randNum];
}

std::vector<frapu::ActionSharedPtr> DiscreteVectorActionSpace::getAllActionsInOrder() const
{
    if (allActionsOrdered_.size() == 0) {
        frapu::ERROR("DiscreteVectorActionSpace: getAllActionsInOrder(): number of actions is 0. Have you forgotten to set the action limits?");
    }
    return allActionsOrdered_;
}

frapu::ActionSharedPtr DiscreteVectorActionSpace::getAction(unsigned int& index) const
{
    if (index > allActionsOrdered_.size()) {
        frapu::ERROR("DiscreteVectorActionSpace: getAction(): action with index " +
                     std::to_string(index) + " requested, but largest index is " +
                     std::to_string(allActionsOrdered_.size()));
    }

    return allActionsOrdered_[index];
}

}
