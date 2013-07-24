#ifndef RANKSELECTION_H
#define RANKSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>

using namespace std;

class RankSelection
{
public:
    RankSelection(){}
    virtual ~RankSelection(){}

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*> _unselected){
        vector<Chromosome*> output;
        
        int max = 0;
        
        for(uint k = 1; k <= population.size(); k++)
            max += k;
    
        while(output.size() < _selectionCount){
            max -= _selectionPool.size();
            output.push_back(selectSingleChromosome(_selectionPool, max);
        }

        unselected = _selectionPool;

        return output;        
    }

    Chromosome* selectSingleChromosome(vector<Chromosome*>& _selectionPool, int _max){
        boost::mt19937 rng(rand());
        boost::uniform_int<> dist(1, _max);
        boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

        int selection = gen();

        int k = -1;

        while(selection > 0)
            selection -= _selectionPool.size() - ++k;

        Chromosome* output = _selectionPool[k];
        _selectionPool.erase(_selectionPool.begin() + k);

        return output;
    }
};

#endif