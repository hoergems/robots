#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>

namespace shared
{
class ObservationSpace
{
public:
    ObservationSpace();
    
    void setDimension(unsigned int dimension);
    
    unsigned int getDimension() const;
    
    void setLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits);
    
    void getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const;
    
private:
    unsigned int dimension_;
    
    std::vector<double> lowerLimits_;
    
    std::vector<double> upperLimits_;
    
    

};

}

#endif
