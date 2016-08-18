#ifndef __ACTION_SPACE_HPP__
#define __ACTION_SPACE_HPP__
#include <string>

namespace shared
{
class ActionSpace
{
public:
    ActionSpace();
    
    /**
     * Returns the type of the action space (discrete, continuous, hybrid)
     */
    virtual std::string getType() const = 0;
    
    void setNumDimensions(unsigned int &numDimensions);
    
    unsigned int getNumDimensions() const;

protected:    
    unsigned int numDimensions_;

};
}

#endif
