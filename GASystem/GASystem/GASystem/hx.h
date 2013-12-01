#ifndef HX_H
#define HX_H

#include "crossover.h"

class HX : public Crossover
{
public:
    HX();
    virtual ~HX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createHX(){
        return new HX();
    }

};

#endif