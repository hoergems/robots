#include <robot_headers/ActionSpace.hpp>

namespace frapu
{
DiscreteVectorActionSpace::DiscreteVectorActionSpace(const ActionSpaceInfo& actionSpaceInfo):
    DiscreteActionSpace(actionSpaceInfo)
{

}

ActionSharedPtr DiscreteVectorActionSpace::sampleUniform(std::default_random_engine* randGen) const
{
    std::vector<double> lowerActionLimits;
    std::vector<double> upperActionLimits;
    static_cast<VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
    std::vector<double> randomActionVec(lowerActionLimits.size());
    for (size_t i = 0; i < lowerActionLimits.size(); i++) {
        unsigned int rand_num = std::uniform_int_distribution<long>(0, 1)(*randGen);
        if (rand_num == 0) {
            randomActionVec[i] = lowerActionLimits[i];
        } else {
            randomActionVec[i] = upperActionLimits[i];
        }
    }

    ActionSharedPtr action = std::make_shared<VectorAction>(randomActionVec);
    return action;
}

std::vector<frapu::ActionSharedPtr> DiscreteVectorActionSpace::getAllActionsInOrder(unsigned int& numStepsPerDimension) const
{
    std::vector<frapu::ActionSharedPtr> allActions;
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    unsigned int numDimensions = getNumDimensions();
    ActionLimitsSharedPtr actionLimits = getActionLimits();
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
        
        allActions.push_back(std::make_shared<frapu::VectorAction>(actionValues));
    }
    
    return allActions;
}

}
