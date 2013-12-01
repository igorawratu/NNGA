#ifndef AX_H
#define AX_H

#include "crossover.h"

class AX : public Crossover
{
public:
    AX();
    virtual ~AX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createAX(){
        return new AX();
    }
};

#endif