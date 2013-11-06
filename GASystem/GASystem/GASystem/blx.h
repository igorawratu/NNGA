#ifndef BLX_H
#define BLX_H

#include "crossover.h"

class BLX : public Crossover
{
public:
    BLX();
    virtual ~BLX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters);
    static Crossover* createBLX(){
        return new BLX();
    }
};

#endif;