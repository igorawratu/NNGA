#include "hx.h"

HX::HX(){}

HX::~HX(){}

vector<Chromosome*> HX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
    assert(_population.size() >= 2);
    
    vector<Chromosome*> offspring;

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    while(offspring.size() < numOffspring){
        vector<Chromosome*> parents = _selectionAlgorithm->execute(_population, 2, vector<Chromosome*>());

        if(gen() > _parameters["CrossoverProbability"]){
            boost::mt19937 rngP(rand());
            boost::uniform_int<> distP(0, 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genParent(rngP, distP);

            offspring.push_back(parents[genParent()]->clone());
        }
        else{
            vector<map<uint, vector<double>>> p1Weights, p2Weights, childWeights;
            p1Weights = parents[0]->fitness() >= parents[1]->fitness() ? parents[0]->getWeightData() : parents[1]->getWeightData();
            p2Weights = parents[1]->fitness() > parents[0]->fitness() ? parents[0]->getWeightData() : parents[1]->getWeightData();

            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetworkWeights;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights;

                    for(uint i = 0; i < iter->second.size(); i++){
                        double r = gen();
                        double childVal = p1Weights[k][iter->first][i] + r * (p1Weights[k][iter->first][i] - p2Weights[k][iter->first][i]);

                        weights.push_back(childVal);
                    }

                    currentNetworkWeights[iter->first] = weights;
                }
                childWeights.push_back(currentNetworkWeights);
            }
            Chromosome* child1 = parents[0]->clone();
            child1->setWeights(childWeights);
            offspring.push_back(child1);
        }
    }

    return offspring;
}