#include "nlrankselection.h"

NLRankSelection::NLRankSelection(){
    V = 0.1;
}
NLRankSelection::~NLRankSelection(){}

void NLRankSelection::calculateRankFitness(vector<Chromosome*> _selectionPool){
    for(uint k = 0; k < _selectionPool.size(); ++k){
        double base = (1 - V);
        double fit = V * pow(base, (double)k);
        mRankFitness.push_back(fit);
    }
}