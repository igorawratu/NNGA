#ifndef MULTIPOINTCROSSOVER_H
#define MULTIPOINTCROSSOVER_H

#include "crossover.h"

class MultipointCrossover : public Crossover
{
public:
    MultipointCrossover();
    virtual ~MultipointCrossover();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createMultipointCrossover(){
        return new MultipointCrossover();
    }

};


#endif