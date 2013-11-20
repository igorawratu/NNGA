#include "randomselection.h"

RandomSelection::RandomSelection(){}
RandomSelection::~RandomSelection(){}

vector<Chromosome*> RandomSelection::execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
    assert(_selectionPool.size() >= _selectionCount);
    
    vector<Chromosome*> output;

    //return _selectionPool;

    while(output.size() < _selectionCount){
        int pos;

        if(_selectionPool.size() > 1){
            boost::mt19937 rng(rand());
            boost::uniform_int<> dist(0, _selectionPool.size() - 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

            pos = gen();
        }
        else pos = 0;

        output.push_back(_selectionPool[pos]);
        _selectionPool.erase(_selectionPool.begin() + pos);
    }

    _unselected = _selectionPool;

    return output;        
}