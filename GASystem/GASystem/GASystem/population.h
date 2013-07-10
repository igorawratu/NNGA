#ifndef POPULATION_H
#define POPULATION_H

#include <vector>
#include <string>

#include "common.h"
#include "chromosome.h"

using namespace std;

class Population
{
public:
    Population(){}

    Population(uint _populationSize){
        mPopulation = _populationSize;
    }

    virtual ~Population(){
        for(int k = 0; k < mPopulation; k++){
            delete mPopulation[k];
            mPopulation[k] = 0;
        }
        mPopulation.clear();
    }
    
    virtual void crossover(string _crossoverType)=0;
    void mutate(string _mutationType){
        for(int k = 0; k < mPopulation.size(); k++)
            mPopulation[k]->mutate(_mutationType);
    }
    virtual void nextGeneration(string _selectionType)=0;

protected:
    vector<Chromosome*> mPopulation;
    uint mPopulationSize;
};

#endif