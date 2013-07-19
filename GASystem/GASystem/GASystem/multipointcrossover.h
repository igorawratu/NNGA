#ifndef MULTIPOINTCROSSOVER_H
#define MULTIPOINTCROSSOVER_H

#include "crossover.h"
#include "algorithmcreator.h"

class MultipointCrossover : public Crossover
{
public:
    MultipointCrossover();
    virtual ~MultipointCrossover();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring){
        Selection* selectionAlgorithm = AlgorithmCreator::instance().createSelectionAlgorithm("Rank", map<string, double>());
        if(!selectionAlgorithm)
            return _population;
        
        



    }

private:

};


#endif