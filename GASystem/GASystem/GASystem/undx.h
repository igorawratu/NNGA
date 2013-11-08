#ifndef UNDX_H
#define UNDX_H

#include "crossover.h"
#include <math.h>

class UNDX : public Crossover
{
public:
    UNDX();
    virtual ~UNDX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters);

    static Crossover* createUNDX(){
        return new UNDX();
    }

};

#endif