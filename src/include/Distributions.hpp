/**
 * Multivariate Normal distribution sampling using C++11 and Eigen matrices.
 *
 * This is taken from http://stackoverflow.com/questions/16361226/error-while-creating-object-from-templated-class
 * (also see http://lost-found-wandering.blogspot.fr/2011/05/sampling-from-multivariate-normal-in-c.html)
 *
 * I have been unable to contact the original author, and I've performed
 * the following modifications to the original code:
 * - removal of the dependency to Boost, in favor of straight C++11;
 * - ability to choose from Solver or Cholesky decomposition (supposedly faster);
 * - fixed Cholesky by using LLT decomposition instead of LDLT that was not yielding
 *   a correctly rotated variance
 *   (see this http://stats.stackexchange.com/questions/48749/how-to-sample-from-a-multivariate-normal-given-the-pt-ldlt-p-decomposition-o )
 */

/**
 * Copyright (c) 2014 by Emmanuel Benazera beniz@droidnik.fr, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

#ifndef __EIGENMULTIVARIATENORMAL_HPP_
#define __EIGENMULTIVARIATENORMAL_HPP_

#include "utils.hpp"
#include <Eigen/Dense>
#include <random>
#include <memory>
#include <cmath>
#include <assert.h>


/*
  We need a functor that can pretend it's const,
  but to be a good random number generator
  it needs mutable state.  The standard Eigen function
  Random() just calls rand(), which changes a global
  variable.
*/
namespace Eigen
{
namespace internal
{
template<typename Scalar>
struct scalar_dist_op {
    static std::mt19937 rng;                        // The uniform pseudo-random algorithm
    mutable std::normal_distribution<Scalar> norm; // gaussian combinator
    mutable std::uniform_real_distribution<double> real_distr;

    EIGEN_EMPTY_STRUCT_CTOR(scalar_dist_op)

    template<typename Index>
    inline const Scalar operator()(Index, Index = 0) const {
        return norm(rng);
    }

    inline void seed(const uint64_t& s) {
        rng.seed(s);
    }

    inline double sampleUniform() {
        return real_distr(rng);
    }
};

template<typename Scalar>
std::mt19937 scalar_dist_op<Scalar>::rng;

/**template<typename Scalar>
struct functor_traits<scalar_dist_op<Scalar> > {
    enum { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false };
};*/

} // end namespace internal
}



namespace Eigen
{

template<typename Scalar>
class Distribution
{
public:
    Matrix< Scalar, Dynamic, 1> _mean;

    Matrix<Scalar, Dynamic, Dynamic> _covar;

    Matrix<Scalar, Dynamic, Dynamic> _covar_inverse;

    uint64_t _seed;

    Distribution() {};

    virtual Eigen::Matrix < Scalar, Eigen::Dynamic, -1 > samples(int nn) = 0;

    virtual double calcPdf(std::vector<Scalar>& x, std::vector<Scalar>& mean) = 0;

    virtual void setCovar(const Matrix<Scalar, Dynamic, Dynamic>& covar) {
        _covar = covar;
    }

    void setMean(const Matrix<Scalar, Dynamic, 1>& mean) {
        _mean = mean;
    }

protected:
    internal::scalar_dist_op<Scalar> randN; // Gaussian functor
};


// Default uniform distribution between 0 and 1
template<typename Scalar>
class UniformDistribution: public Distribution<Scalar>
{
public:
    UniformDistribution():
        Distribution<Scalar>() {

    }
};

template<typename Scalar>
class WeightedDiscreteDistribution: public Distribution<Scalar>
{
public:
    WeightedDiscreteDistribution():
        Distribution<Scalar>(),
        weightedElements_(),
        weights_(),
        hashValues_() {

    }

    void setElements(std::vector<std::pair<std::vector<Scalar>, double>>& elements) {
        double weight_sum = 0.0;
	size_t dim = elements[0].first.size();
        weights_ = std::vector<Scalar>(elements.size());
	hashValues_ = std::vector<size_t>(elements.size());
        for (size_t i = 0; i < elements.size(); i++) {
            weight_sum += elements[i].second;
            weights_[i] = elements[i].second;
        }

        weightedElements_ = elements;
	size_t hashValue;
        for (size_t i = 0; i < elements.size(); i++) {
            weightedElements_[i].second /= weight_sum;
	    
	    hashValue = 0;
	    for (auto &k: elements[i].first) {
		robotutils::hash_combine(hashValue, k);
	    }
	    
	    hashValues_[i] = hashValue;
        }
    }

    Matrix < Scalar, Eigen::Dynamic, -1 > samples(int nn) override {
	size_t numElements = weightedElements_.size();
	assert(numElements > 0 && "WeightedDiscreteDistribution: No elements set!");
	size_t dim = weightedElements_[0].first.size();
        Matrix < Scalar, Eigen::Dynamic, -1 > sampleMatrix(dim, nn);
        for (size_t i = 0; i < nn; i++) {
            double rand_num = this->randN.sampleUniform();
            unsigned int index = 0;
            for (size_t j = 0; j < weights_.size(); j++) {
                if (rand_num < weights_[j]) {
                    index = j;
                    break;
                }

                rand_num -= weights_[j];
            }
            
            for (size_t j = 0; j < dim; j++) {
		sampleMatrix(j, i) = weightedElements_[index].first[j];
	    }
        }

        return sampleMatrix;
    }

    double calcPdf(std::vector<Scalar>& x, std::vector<Scalar>& mean) override {
        double value = x[0];
	size_t hashValue = 0;
	for (auto &k: x) {
	    robotutils::hash_combine(hashValue, k);
	}
	
	size_t index = 0;
	bool indexFound = false;
	for (size_t i = 0; i < hashValues_.size(); i++) {
	    if (hashValue == hashValues_[i]) {
		index = i;
		indexFound = true;
                break;		
	    }
	}
	if (indexFound) {
	    return weightedElements_[index].second;
	}
	
	return 0.0;
	
        /**for (auto & k : weightedElements_) {
            if (k.first == value) {
                return k.second;
            }
        }

        return 0.0;*/
    }

private:
    std::vector<std::pair<std::vector<Scalar>, double>> weightedElements_;

    std::vector<double> weights_;
    
    std::vector<size_t> hashValues_;
};

}

namespace Eigen
{
/**
  Find the eigen-decomposition of the covariance matrix
  and then store it for sampling from a multi-variate normal
*/
template<typename Scalar>
class EigenMultivariateNormal: public Distribution<Scalar>
{

    double normal_dist_term1_;
    Matrix<Scalar, Dynamic, Dynamic> _transform;
    bool _use_cholesky;
    SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic> > _eigenSolver; // drawback: this creates a useless eigenSolver when using Cholesky decomposition, but it yields access to eigenvalues and vectors

public:
    EigenMultivariateNormal(const Matrix<Scalar, Dynamic, 1>& mean, const Matrix<Scalar, Dynamic, Dynamic>& covar,
                            const bool use_cholesky = false, const uint64_t& seed = std::mt19937::default_seed):
        Distribution<Scalar>(),
        normal_dist_term1_(0.0),
        _use_cholesky(use_cholesky) {
        this->randN.seed(seed);
        this->setMean(mean);
        setCovar(covar);
        this->_seed = seed;

        double det = covar.determinant();
        normal_dist_term1_ = 1.0 / std::sqrt(std::pow(2.0 * M_PI, covar.rows()) * det);
    }

    double calcPdf(std::vector<Scalar>& x, std::vector<Scalar>& mean) override {
        VectorXd mu(mean.size());
        VectorXd s(x.size());
        for (size_t i = 0; i < x.size(); i++) {
            mu(i) = mean[i];
            s(i) = x[i];
        }

        double term2 = std::exp((-1.0 / 2.0) * (s - mu).transpose() * this->_covar_inverse * (s - mu));
        double pdf = normal_dist_term1_ * term2;
        return pdf;
    }

    void setCovar(const Matrix<Scalar, Dynamic, Dynamic>& covar) override {
        this->_covar = covar;
        this->_covar_inverse = this->_covar.inverse();

        // Assuming that we'll be using this repeatedly,
        // compute the transformation matrix that will
        // be applied to unit-variance independent normals

        if (_use_cholesky) {
            Eigen::LLT<Eigen::Matrix<Scalar, Dynamic, Dynamic> > cholSolver(this->_covar);
            // We can only use the cholesky decomposition if
            // the covariance matrix is symmetric, pos-definite.
            // But a covariance matrix might be pos-semi-definite.
            // In that case, we'll go to an EigenSolver
            if (cholSolver.info() == Eigen::Success) {
                // Use cholesky solver
                _transform = cholSolver.matrixL();
            } else {
                throw std::runtime_error("Failed computing the Cholesky decomposition. Use solver instead");
            }
        } else {
            _eigenSolver = SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic> >(this->_covar);
            _transform = _eigenSolver.eigenvectors() * _eigenSolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
        }
    }

    /// Draw nn samples from the gaussian and return them
    /// as columns in a Dynamic by nn matrix
    Matrix < Scalar, Dynamic, -1 > samples(int nn) {
        return (_transform * Matrix < Scalar, Dynamic, -1 >::NullaryExpr(this->_covar.rows(), nn, this->randN)).colwise() + this->_mean;
    }
}; // end class EigenMultivariateNormal
} // end namespace Eigen
#endif

