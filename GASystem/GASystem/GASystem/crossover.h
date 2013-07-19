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

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring)=0;
    void setParameters(map<string, double> _parameters){mParameters = _parameters;}

protected:
    map<string, double> mParameters;
};

#endif