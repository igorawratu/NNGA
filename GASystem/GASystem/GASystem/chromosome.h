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

    virtual void mutate(string _mutationType, map<string, double>& _parameters)=0;

    virtual vector<map<uint, vector<double>>> getWeightData()=0;
    virtual vector<map<uint, NeuronInfo>> getFullStructureData()=0;
    virtual void setWeights(vector<map<uint, vector<double>>>& _weights)=0;
    virtual void setStructure(vector<map<uint, NeuronInfo>>& _struct)=0;
    virtual Chromosome* clone()=0;

    double& fitness(){return mFitness;}    

protected:
    double mFitness;
};

#endif