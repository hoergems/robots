#ifndef __ACT_SPACE_HPP__
#define __ACT_SPACE_HPP__
#include <string>
#include <vector>
#include <iostream>
#include <memory>

namespace shared
{

// Normalization functor
struct normalize {
public:
    normalize(std::vector<double>& lowerActionLimits,
              std::vector<double>& upperActionLimits):
        lowerActionLimits_(lowerActionLimits),
        upperActionLimits_(upperActionLimits) {

    }

    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) = 0;

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) = 0;

protected:
    std::vector<double> lowerActionLimits_;
    std::vector<double> upperActionLimits_;
};

struct standardNormalize: public normalize {
public:
    standardNormalize(std::vector< double >& lowerActionLimits,
                      std::vector< double >& upperActionLimits):
        normalize(lowerActionLimits, upperActionLimits) {

    }

    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) override {
        a2.clear();
        a2.resize(a1.size());
        for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
            a2[i] = a1[i] * (upperActionLimits_[i] - lowerActionLimits_[i]) + lowerActionLimits_[i];
        }
    }

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) override {
        a2.clear();
        a2.resize(a1.size());
        for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
            a2[i] = (a1[i] - lowerActionLimits_[i]) / (upperActionLimits_[i] - lowerActionLimits_[i]);
        }
    }
};

struct nullNormalize: public normalize {
public:
    nullNormalize(std::vector< double >& lowerActionLimits,
                  std::vector< double >& upperActionLimits):
        normalize(lowerActionLimits, upperActionLimits) {

    }
    
    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) override {
        a2 = a1;
    }

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) override {
        a2 = a1;
    }

};

class ActionSpace
{
public:
    ActionSpace(bool normalizedActionSpace);

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
    
    bool isNormalized() const;

protected:
    unsigned int numDimensions_;

    std::vector<double> lowerActionLimits_;

    std::vector<double> upperActionLimits_;
    
    bool normalizedActionSpace_;

    std::unique_ptr<shared::normalize> actionNormalizer_;

};
}

#endif
