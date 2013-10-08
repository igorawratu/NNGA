#ifndef RANKEXPONENTIALSELECTION_H
#define RANKEXPONENTIALSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>
#include <math.h> 

using namespace std;

class RankExponentialSelection : public Selection
{
public:
    RankExponentialSelection(){}
    virtual ~RankExponentialSelection(){}

    static Selection* createRankExponentialSelection(){
        return new RankExponentialSelection();
    }

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
        assert(_selectionPool.size() >= _selectionCount);
        
        vector<Chromosome*> output;
        
        long max = 0;
        for(long k = 0; k < _selectionPool.size(); k++)
            max += powl(2, k + 1);
    
        while(output.size() < _selectionCount){
            output.push_back(selectSingleChromosome(_selectionPool, max));
            max -= powl(2, _selectionPool.size() + 1);
        }

        _unselected = _selectionPool;

        return output;        
    }


private:
    Chromosome* selectSingleChromosome(vector<Chromosome*>& _selectionPool, long _max){
        boost::mt19937 rng(rand());
        boost::uniform_int<> dist(1, _max);
        boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

        int selection = gen();

        int k = -1;

        while(selection > 0)
            selection -= powl(2, _selectionPool.size() - ++k);

        Chromosome* output = _selectionPool[k];
        _selectionPool.erase(_selectionPool.begin() + k);

        return output;
    }
};

#endif