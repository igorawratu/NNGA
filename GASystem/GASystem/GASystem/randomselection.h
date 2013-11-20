#ifndef RANDOMSELECTION_H
#define RANDOMSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>
#include <math.h> 

using namespace std;

class RandomSelection : public Selection
{
public:
    RandomSelection();
    virtual ~RandomSelection();

    static Selection* createRandomSelection(){
        return new RandomSelection();
    }

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected);
};

#endif