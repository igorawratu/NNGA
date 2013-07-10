#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H

#include <vector>
#include <string>

#include "population.h"
#include "simulation.h"

using namespace std;

class GeneticAlgorithm
{
public:
    GeneticAlgorithm(){}
    virtual GeneticAlgorithm(){}

    virtual Solution train(string _simulation);

protected:
    vector<Population*> populations;
};

#endif