#ifndef NLRANKSELECTION_H
#define NLRANKSELECTION_H

#include "rankselection.h"

using namespace std;

class NLRankSelection : public RankSelection
{
public:
    NLRankSelection();
    virtual ~NLRankSelection();

    static Selection* createNLRankSelection(){
        return new NLRankSelection();
    }


protected:
    virtual void calculateRankFitness(vector<Chromosome*> _selectionPool);
private:
    double V;
};

#endif