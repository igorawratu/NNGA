#ifndef SELECTION_H
#define SELECTION_H

#include <vector>

#include "common.h"
#include "chromosome.h"

using namespace std;

class Selection
{
public:
    Selection(){}
    virtual ~Selection(){}

    virtual vector<Chromosome*> execute(vector<Chromosome*>& _selectionPool, uint _selectionCount)=0;
};

#endif