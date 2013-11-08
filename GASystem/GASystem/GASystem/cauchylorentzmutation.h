#ifndef CAUCHYLORENTZMUTATION_H
#define CAUCHYLORENTZMUTATION_H

#include <map>

#include "mutation.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

using namespace std;

class CauchyLorentzMutation : public Mutation
{
public:
    CauchyLorentzMutation();
    virtual ~CauchyLorentzMutation();

    static Mutation* createCauchyLorentzMutation(){
        return new CauchyLorentzMutation();
    }

    virtual void execute(vector<double>& _weights, map<string, double>& _parameters);
    
};

#endif