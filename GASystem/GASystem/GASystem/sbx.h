#ifndef SBX_H
#define SBX_H

#include "crossover.h"

class SBX : public Crossover
{
public:
    SBX();
    virtual ~SBX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createSBX(){
        return new SBX();
    }
};

#endif