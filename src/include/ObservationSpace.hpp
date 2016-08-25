#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>
#include <string>
#include "utils.hpp"

namespace shared
{
    
struct ObservationSpaceInfo {
public:
    ObservationSpaceInfo() {}
    
    // The observation type ('discrete' or 'continuous')
    std::string observationType;
    
    // Contains additional information (e.g. 'linear' or 'nonlinear')
    std::string observationModelInfo;
};

class ObservationSpace
{
public:
    ObservationSpace(const shared::ObservationSpaceInfo &observationSpaceInfo);
    
    void setDimension(unsigned int dimension);
    
    unsigned int getDimension() const;
    
    const shared::ObservationSpaceInfo getObservationSpaceInfo() const;
    
private:
    unsigned int dimension_;
    
    shared::ObservationSpaceInfo observationSpaceInfo_;
};

}

#endif
