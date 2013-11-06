#ifndef RANKSELECTION_H
#define RANKSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>
#include <math.h>

using namespace std;

class RankSelection : public Selection
{
public:
    RankSelection();
    virtual ~RankSelection();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected);


protected:
    virtual void calculateRankFitness(vector<Chromosome*> _selectionPool) = 0;

private:
    Chromosome* selectSingleChromosome(vector<Chromosome*>& _selectionPool);

    void calculateMaxFitness();

protected:
    vector<double> mRankFitness;
    double mMaxFit;
};

#endif