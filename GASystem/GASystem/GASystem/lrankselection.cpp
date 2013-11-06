#include "lrankselection.h"

LRankSelection::LRankSelection(){
    SP = 2;
}

LRankSelection::~LRankSelection(){}

void LRankSelection::calculateRankFitness(vector<Chromosome*> _selectionPool){
    for(uint k = 0; k < _selectionPool.size(); ++k){
        double fit = 2 - SP + 2 * (SP - 1) * ((double)(_selectionPool.size() - k) / (double)_selectionPool.size());
        mRankFitness.push_back(fit);
    }
}