#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include <vector>
#include <iostream>
#include <map>

#include "common.h"
#include "neuralnetwork.h"

using namespace std;

class Chromosome
{
public:
    Chromosome(){}
    virtual ~Chromosome(){}

    virtual void mutate(MutationAlgorithm* _mutationAlgorithm)=0;
    virtual void getFitness()=0;

};

#endif