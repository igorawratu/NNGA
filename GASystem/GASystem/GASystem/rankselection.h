#ifndef RANKSELECTION_H
#define RANKSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>

using namespace std;

class RankSelection : public Selection
{
public:
    RankSelection(){}
    virtual ~RankSelection(){}

    static Selection* createRankSelection(){
        return new RankSelection();
    }

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
        assert(_selectionPool.size() >= _selectionCount);
        
        vector<Chromosome*> output;
        
        int max = (_selectionPool.size() * (_selectionPool.size() + 1)) / 2;
    
        while(output.size() < _selectionCount){
            output.push_back(selectSingleChromosome(_selectionPool, max));
            max -= (_selectionPool.size() + 1);
        }

        _unselected = _selectionPool;

        return output;        
    }


private:
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