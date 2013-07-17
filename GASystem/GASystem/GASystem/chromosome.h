#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include <iostream>
#include <map>
#include <string>

#include "common.h"
#include "neuralnetwork.h"

using namespace std;

class Chromosome
{
public:
    Chromosome(){}
    virtual ~Chromosome(){}

    virtual void mutate(string _mutationType)=0;

    virtual vector<map<uint, vector<double>>> getWeightData()=0;
    virtual vector<map<uint, NeuronInfo>> getFullStructureData()=0;

    double& fitness(){return mFitness;}

protected:
    double mFitness;
};

#endif