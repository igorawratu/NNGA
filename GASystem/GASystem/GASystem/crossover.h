#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "selectionfactory.h"
#include "chromosome.h"
#include "common.h"

#include <vector>
#include <map>

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

using namespace std;

class Crossover
{
public:
    Crossover(){}
    virtual ~Crossover(){}

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters)=0;
};

#endif