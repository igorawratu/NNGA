#ifndef MULTIPOINTCROSSOVER_H
#define MULTIPOINTCROSSOVER_H

#include "crossover.h"
#include "algorithmcreator.h"
#include "chromosome.h"
#include "common.h"

#include <vector>
#include <map.h>

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

class MultipointCrossover : public Crossover
{
public:
    MultipointCrossover();
    virtual ~MultipointCrossover();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring){

        Selection* selectionAlgorithm = AlgorithmCreator::instance().createSelectionAlgorithm("Rank", map<string, double>());
        if(!selectionAlgorithm)
            return _population;
        
        vector<Chromosome*> offspring;

        boost::mt19937 rng(rand());
        boost::uniform_real<double> dist(0, 1);
        boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

        while(offspring.size() < numOffspring){
            vector<Chromosome*> parents = selectionAlgorithm->execute(_population, 2);

            vector<map<uint, vector<double>>> p1Weights, p2Weights, childWeights;
            p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData();

            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetworkWeights;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights;

                    for(uint i = 0; i < iter->second.size(); i++)
                        weights.push_back(gen() < 0.5 ? iter->second[i] : p2Weights[iter->first][i]);

                    currentNetworkWeights[iter->first] = weights;
                }
                childWeights.push_back(currentNetworkWeights);
            }
            Chromosome* child = parents[0]->clone();
            child->setWeights(childWeights);
            offspring.push_back(child);
        }

        return offspring;
    }

};


#endif