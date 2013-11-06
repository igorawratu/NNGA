#include "quadraticrankselection.h"

QuadraticRankSelection::QuadraticRankSelection(){
}
QuadraticRankSelection::~QuadraticRankSelection(){}

void QuadraticRankSelection::calculateRankFitness(vector<Chromosome*> _selectionPool){
    for(uint k = 0; k < _selectionPool.size(); ++k){
        double base = _selectionPool.size() - k;
        double fit = pow(base, 2);
        mRankFitness.push_back(fit);
    }
}