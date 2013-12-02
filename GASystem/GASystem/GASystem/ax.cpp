#include "ax.h"

AX::AX(){}

AX::~AX(){}

vector<Chromosome*> AX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
    assert(_population.size() >= 2);
    
    vector<Chromosome*> offspring;

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    while(offspring.size() < numOffspring){
        vector<Chromosome*> parents = _selectionAlgorithm->execute(_population, 2, vector<Chromosome*>());

        if(gen() > _parameters["CrossoverProbability"]){
            boost::mt19937 rngP(rand());
            boost::uniform_int<> distP(0, parents.size() - 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genParent(rngP, distP);

            offspring.push_back(parents[genParent()]->clone());
        }
        else{
            vector<map<uint, vector<double>>> p1Weights, p2Weights, child1Weights, child2Weights;
            p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData();

            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetwork1Weights, currentNetwork2Weights;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights1, weights2;

                    for(uint i = 0; i < iter->second.size(); i++){
                        double alpha = gen();

                        double child1Val = alpha * p1Weights[k][iter->first][i] + (1 - alpha) * p2Weights[k][iter->first][i];
                        double child2Val = (1 - alpha) * p1Weights[k][iter->first][i] + alpha * p2Weights[k][iter->first][i];

                        weights1.push_back(child1Val);
                        weights2.push_back(child2Val);
                    }

                    currentNetwork1Weights[iter->first] = weights1;
                    currentNetwork2Weights[iter->first] = weights2;
                }
                child1Weights.push_back(currentNetwork1Weights);
                child2Weights.push_back(currentNetwork2Weights);
            }
            Chromosome* child1 = parents[0]->clone();
            Chromosome* child2 = parents[0]->clone();
            child1->setWeights(child1Weights);
            child2->setWeights(child2Weights);
            offspring.push_back(child1);
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