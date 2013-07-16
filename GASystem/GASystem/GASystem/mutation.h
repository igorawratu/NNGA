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
    void setParameters(map<string, double> _parameters){mParameters = _parameters;}

protected:
    map<string, double> mParameters;
};

#endif