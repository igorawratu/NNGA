#ifndef SPX_H
#define SPX_H

#include "crossover.h"

class SPX : public Crossover
{
public:
    SPX();
    virtual ~SPX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters);
    static Crossover* createSPX(){
        return new SPX();
    }

    void quicksort(vector<Chromosome*>& elements, int left, int right);
};

#endif