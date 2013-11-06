#ifndef QUADRATICRANKSELECTION_H
#define QUADRATICRANKSELECTION_H

#include "rankselection.h"

class QuadraticRankSelection : public RankSelection
{
public:
    QuadraticRankSelection();
    virtual ~QuadraticRankSelection();

    static Selection* createQuadraticRankSelection(){
        return new QuadraticRankSelection();
    }


protected:
    virtual void calculateRankFitness(vector<Chromosome*> _selectionPool);
};

#endif