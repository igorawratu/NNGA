#include "neatx.h"

NEATX::NEATX(){}

NEATX::~NEATX(){}

vector<Chromosome*> NEATX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
    assert(_population.size() > 0);

    vector<Chromosome*> offspring;
    if(_population.size() == 1){
        for(uint k = 0; k < numOffspring; ++k){
            offspring.push_back(_population[0]->clone());
        }
        return offspring;
    }

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    for(uint k = 0; k < numOffspring; ++k){
        vector<Chromosome*> parents = _selectionAlgorithm->execute(_population, 2, vector<Chromosome*>());

        if(gen() > _parameters["CrossoverProbability"]){
            boost::mt19937 rngP(rand());
            boost::uniform_int<> distP(0, 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genParent(rngP, distP);

            offspring.push_back(parents[genParent()]->clone());
        }
        else{
            Chromosome* child = dynamic_cast<NeatChromosome*>(parents[0])->produceOffspring(parents[1]);
            offspring.push_back(child);
        }
    }

    return offspring;
}