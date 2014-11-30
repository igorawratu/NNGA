#ifndef NEATX_H
#define NEATX_H

#include "crossover.h"
#include "neatchromosome.h"

class NEATX : public Crossover
{
public:
    NEATX();
    virtual ~NEATX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createNEATX(){
        return new NEATX();
    }

};

#endif