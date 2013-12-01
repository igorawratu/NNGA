#ifndef TPX_H
#define TPX_H

#include "crossover.h"

class TPX : public Crossover
{
public:
    TPX();
    virtual ~TPX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createTPX(){
        return new TPX();
    }

};

#endif