#ifndef OPX_H
#define OPX_H

#include "crossover.h"

class OPX : public Crossover
{
public:
    OPX();
    virtual ~OPX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);
    static Crossover* createOPX(){
        return new OPX();
    }

};

#endif