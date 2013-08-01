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

    virtual void execute(vector<double>& _weights, map<string, double>& _parameters)=0;
};

#endif