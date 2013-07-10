#ifndef MUTATION_H
#define MUTATION_H

#include <vector>

#include "common.h"

using namespace std;

class Mutation
{
public:
    Mutation(){}
    virtual ~Mutation(){}

    virtual void execute(vector<double>& _weights)=0;
    virtual void execute(vector<float>& _weights)=0;

};

#endif