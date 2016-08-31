#include <robot_headers/ContinuousVectorActionSpace.hpp>

namespace frapu
{
ContinuousVectorActionSpace::ContinuousVectorActionSpace(const ActionSpaceInfo &actionSpaceInfo):
    ContinuousActionSpace(actionSpaceInfo)
{

}

ActionSharedPtr ContinuousVectorActionSpace::sampleUniform(std::default_random_engine* randGen) const
{
    std::vector<double> lowerActionLimits;
    std::vector<double> upperActionLimits;
    static_cast<VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
    std::vector<double> randomActionVec(lowerActionLimits.size());
    for (size_t i = 0; i < lowerActionLimits.size(); i++) {
        std::uniform_real_distribution<double> uniform_dist(lowerActionLimits[i], upperActionLimits[i]);
        double rand_num = uniform_dist(*randGen);
        randomActionVec[i] = rand_num;
    }
    
    ActionSharedPtr action = std::make_shared<VectorAction>(randomActionVec);
    return action;
}

}
