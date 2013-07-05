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

    //templatize this shit to allow for different mutation algorithms
    virtual void mutate()=0;
    

private:
    double mFitness;

};

#endif