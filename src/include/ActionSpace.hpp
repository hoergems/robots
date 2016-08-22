#ifndef __ACT_SPACE_HPP__
#define __ACT_SPACE_HPP__
#include <string>
#include <vector>

namespace shared
{

/**void normalizeAction(std::vector<double>& action,
                     std::vector<double>& lowerActionLimits,
                     std::vector<double>& upperActionLimits,
                     std::vector<double>& normalizedAction)
{
    normalizedAction.clear();
    normalizedAction.resize(action.size());
    for (size_t i = 0; i < lowerActionLimits.size(); i++) {
        normalizedAction[i] = (action[i] - lowerActionLimits[i]) / (upperActionLimits[i] - lowerActionLimits[i]);
    }
};

void denormalizeAction(std::vector<double>& normalizedAction,
                       std::vector<double>& lowerActionLimits,
                       std::vector<double>& upperActionLimits,
                       std::vector<double>& action)
{
    action.clear();
    action.resize(action.size());
    for (size_t i = 0; i < lowerActionLimits.size(); i++) {
        action[i] = normalizedAction[i] * (upperActionLimits[i] - lowerActionLimits[i]) + lowerActionLimits[i];
    }
};*/

class ActionSpace
{
public:
    ActionSpace();

    /**
     * Returns the type of the action space (discrete, continuous, hybrid)
     */
    virtual std::string getType() const = 0;

    void setNumDimensions(unsigned int& numDimensions);

    void setActionLimits(std::vector<double>& lowerActionLimits,
                         std::vector<double>& upperActionLimits);

    unsigned int getNumDimensions() const;

    void normalizeAction(std::vector<double>& action,
                         std::vector<double>& normalizedAction);

    void denormalizeAction(std::vector<double>& action,
                           std::vector<double>& normalizedAction);

protected:
    unsigned int numDimensions_;

    std::vector<double> lowerActionLimits_;

    std::vector<double> upperActionLimits_;

};
}

#endif
