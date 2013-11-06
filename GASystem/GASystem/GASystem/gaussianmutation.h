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
    GaussianMutation();
    virtual ~GaussianMutation();

    static Mutation* createGaussianMutation(){
        return new GaussianMutation();
    }

    virtual void execute(vector<double>& _weights, map<string, double>& _parameters);
    
};

#endif