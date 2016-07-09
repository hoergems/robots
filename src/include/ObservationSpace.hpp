#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>
#include <string>

namespace shared
{
class ObservationSpace
{
public:
    ObservationSpace(std::string &observationType);
    
    void setDimension(unsigned int dimension);
    
    unsigned int getDimension() const;
    
    std::string getObservationType() const;
    
    void setLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits);
    
    void getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const;
    
private:
    unsigned int dimension_;
    
    std::vector<double> lowerLimits_;
    
    std::vector<double> upperLimits_;
    
    std::string observationType_;
    
    

};

}

#endif
