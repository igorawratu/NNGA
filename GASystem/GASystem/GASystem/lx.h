#ifndef LX_H
#define LX_H

#include "crossover.h"
#include <math.h>

class LX : public Crossover
{
public:
    LX();
    virtual ~LX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createLX(){
        return new LX();
    }
};

#endif