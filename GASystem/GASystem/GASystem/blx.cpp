#include "blx.h"

BLX::BLX(){}

BLX::~BLX(){}

vector<Chromosome*> BLX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters){
    assert(_population.size() >= 2);

    Selection* selectionAlgorithm = SelectionFactory::instance().create("QuadraticRankSelection");
    if(!selectionAlgorithm)
        return _population;
    
    vector<Chromosome*> offspring;

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    while(offspring.size() < numOffspring){
        vector<Chromosome*> parents = selectionAlgorithm->execute(_population, 2, vector<Chromosome*>());

        vector<map<uint, vector<double>>> p1Weights, p2Weights, childWeights;
        p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData();

        for(uint k = 0; k < p1Weights.size(); k++){
            map<uint, vector<double>> currentNetworkWeights;
            for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                vector<double> weights;

                for(uint i = 0; i < iter->second.size(); i++){
                    double alpha = 0.5;
                    double eqWeight = (1 + 2*alpha) * gen() - alpha;

                    double childVal = (1 - eqWeight) * iter->second[i] + eqWeight * p2Weights[k][iter->first][i];
                    weights.push_back(childVal);
                }

                currentNetworkWeights[iter->first] = weights;
            }
            childWeights.push_back(currentNetworkWeights);
        }
        Chromosome* child = parents[0]->clone();
        child->setWeights(childWeights);
        offspring.push_back(child);
    }

    delete selectionAlgorithm;

    return offspring;
}