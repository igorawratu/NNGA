#ifndef GAUSSIANMUTATION_H
#define GAUSSIANMUTATION_H

#include <map>

#include "mutation.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

using namespace std;

class GaussianMutation : public Mutation
{
public:
    GaussianMutation(){}
    virtual ~GaussianMutation(){}

    virtual void execute(vector<double>& _weights){
        double mutationProbability, deviation, maxConstraint, minConstraint;

        if(!getParameter(mParameters, mutationProbability, "MutationProbability"))
            return;

        if(!getParameter(mParameters, deviation, "Deviation"))
            return;

        if(!getParameter(mParameters, maxConstraint, "MaxConstraint"))
            return;

        if(!getParameter(mParameters, minConstraint, "MinConstraint"))
            return;

        boost::mt19937 mRNGMutationProb(rand()), mRNGMutation(rand());

        boost::uniform_real<double> mutationProbDist(0, 1);
        boost::normal_distribution<> mMutationDist(0, deviation);

        boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genMutationProb(mRNGMutationProb, mutationProbDist);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > genMutation(mRNGMutation, mMutationDist);

        for(uint k = 0; k < _weights.size(); k++){
            if(genMutationProb() < mutationChance)
                _weights[k] += genMutation();
            if(_weights[k] > maxConstraint)
                _weights[k] = maxConstraint;
            if(_weights[k] < minConstraint)
                _weights[k[ = minConstraint;
        }
    }
    
    virtual void execute(vector<float>& _weights){
        double mutationProbability, deviation, maxConstraint, minConstraint;

        if(!getParameter(mParameters, mutationProbability, "MutationProbability"))
            return;

        if(!getParameter(mParameters, deviation, "Deviation"))
            return;

        if(!getParameter(mParameters, maxConstraint, "MaxConstraint"))
            return;

        if(!getParameter(mParameters, minConstraint, "MinConstraint"))
            return;

        boost::uniform_real<float> mutationProbDist(0, 1);
        boost::normal_distribution<> mMutationDist(0, deviation);

        boost::variate_generator<boost::mt19937, boost::uniform_real<float>> genMutationProb(mRNGMutationProb, mutationProbDist);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > genMutation(mRNGMutation, mMutationDist);

        for(uint k = 0; k < _weights.size(); k++){
            if(genMutationProb() < mutationChance)
                _weights[k] += genMutation();
            if(_weights[k] > maxConstraint)
                _weights[k] = maxConstraint;
            if(_weights[k] < minConstraint)
                _weights[k[ = minConstraint;
        }
    }
    

};

#endif