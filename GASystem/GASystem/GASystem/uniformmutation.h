#ifndef UNIFORMMUTATION_H
#define UNIFORMMUTATION_H

#include <map>

#include "mutation.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

using namespace std;

class UniformMutation : public Mutation
{
public:
    UniformMutation();
    virtual ~UniformMutation();

    static Mutation* createUniformMutation(){
        return new UniformMutation();
    }

    virtual void execute(vector<double>& _weights, map<string, double>& _parameters);
    
};

#endif