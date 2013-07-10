#ifndef CROSSOVER_H
#define CROSSOVER_H

#include <vector>

#include "common.h"
#include "chromosome.h"

using namespace std;

class Crossover
{
public:
    Crossover(){}
    virtual ~Crossover(){}

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population)=0; 
};

#endif