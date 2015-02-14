#include "tournamentselection.h"

TournamentSelection::TournamentSelection(){}
TournamentSelection::~TournamentSelection(){}

vector<Chromosome*> TournamentSelection::execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
    assert(_selectionPool.size() >= _selectionCount);
    
    vector<Chromosome*> output;

    while(output.size() < _selectionCount)
        output.push_back(runTournament(_selectionPool, 4));

    _unselected = _selectionPool;

    return output;        
}

Chromosome* TournamentSelection::runTournament(vector<Chromosome*>& _selectionPool, uint _tournamentSize){
    assert(_tournamentSize <= _selectionPool.size() && _tournamentSize > 0);

    boost::mt19937 rng(rand());
    boost::uniform_int<> dist(0, _selectionPool.size() - 1);
    boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

    //selects tournament participants
    vector<uint> tourneyPool;
    for(uint k = 0; k < _tournamentSize; k++){
        bool add = false;
        int pos;
        while(!add){
            add = true;
            pos = gen();
            for(uint i = 0; i < tourneyPool.size(); i++){
                if(tourneyPool[i] == pos){
                    add = false;
                    break;
                }
            }

        }
        tourneyPool.push_back(pos);
    }
    
    //run tournament
    uint bestPos = 0;
    double bestFit = 99999999999;
    for(uint k = 0; k < tourneyPool.size(); k++){
        if(_selectionPool[tourneyPool[k]]->fitness() < bestFit){
            bestFit = _selectionPool[tourneyPool[k]]->fitness();
            bestPos = k;
        }
    }

    Chromosome* output = _selectionPool[tourneyPool[0]];
    _selectionPool.erase(_selectionPool.begin() + tourneyPool[0]);

    return output;
}