#include "lx.h"

LX::LX(){}

LX::~LX(){}

vector<Chromosome*> LX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
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
            vector<map<uint, vector<double>>> p1Weights, p2Weights, child1Weights, child2Weights;
            p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData();

            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetworkWeights1, currentNetworkWeights2;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights1;
                    vector<double> weights2;

                    for(uint i = 0; i < iter->second.size(); i++){
                        double u = gen();
                        double a = 0;
                        double b = 0.25;

                        double beta = u < 0.5 ? a + b * log(2 * u) : a - b * log(2 - 2 * u);

                        double childVal1 = p1Weights[k][iter->first][i] + beta * fabs(p1Weights[k][iter->first][i] - p2Weights[k][iter->first][i]);
                        double childVal2 = p2Weights[k][iter->first][i] + beta * fabs(p1Weights[k][iter->first][i] - p2Weights[k][iter->first][i]);

                        weights1.push_back(childVal1);
                        weights2.push_back(childVal2);
                    }

                    currentNetworkWeights1[iter->first] = weights1;
                    currentNetworkWeights2[iter->first] = weights2;
                }
                child1Weights.push_back(currentNetworkWeights1);
                child2Weights.push_back(currentNetworkWeights2);
            }
            Chromosome* child1 = parents[0]->clone();
            child1->setWeights(child1Weights);
            offspring.push_back(child1);
            Chromosome* child2 = parents[0]->clone();
            child2->setWeights(child2Weights);
            offspring.push_back(child2);
        }
    }

    while(offspring.size() > numOffspring){
        Chromosome* toRemove = offspring[0];
        delete toRemove;
        offspring.erase(offspring.begin());
    }

    return offspring;
}