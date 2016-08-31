#include <robot_headers/DiscreteVectorActionSpace.hpp>

namespace frapu
{
DiscreteVectorActionSpace::DiscreteVectorActionSpace(bool normalizedActionSpace):
    DiscreteVectorActionSpace(normalizedActionSpace)
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
        }
        else {
            randomActionVec[i] = upperActionLimits[i];
        }
    }
    
    ActionSharedPtr action = std::make_shared<VectorAction>(randomActionVec);
    return action;
}

}
