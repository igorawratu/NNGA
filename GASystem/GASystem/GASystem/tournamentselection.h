#ifndef TOURNAMENTSELECTION_H
#define TOURNAMENTSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>
#include <math.h> 

using namespace std;

class TournamentSelection : public Selection
{
public:
    TournamentSelection();
    virtual ~TournamentSelection();

    static Selection* createTournamentSelection(){
        return new TournamentSelection();
    }

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected);


private:
    Chromosome* runTournament(vector<Chromosome*>& _selectionPool, uint _tournamentSize);
};

#endif