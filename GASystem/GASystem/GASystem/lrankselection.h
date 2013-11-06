#ifndef LRANKSELECTION_H
#define LRANKSELECTION_H

#include "rankselection.h"

using namespace std;

class LRankSelection : public RankSelection
{
public:
    LRankSelection();
    virtual ~LRankSelection();

    static Selection* createLRankSelection(){
        return new LRankSelection();
    }


protected:
    virtual void calculateRankFitness(vector<Chromosome*> _selectionPool);

private:
    double SP;
};

#endif