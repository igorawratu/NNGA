#include "tpx.h"

TPX::TPX(){}

TPX::~TPX(){}

vector<Chromosome*> TPX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
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
            vector<double> p1, p2, c;
            p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData();

            for(uint k = 0; k < p1Weights.size(); ++k){
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    for(uint i = 0; i < iter->second.size(); i++){
                        p1.push_back(p1Weights[k][iter->first][i]);
                        p2.push_back(p2Weights[k][iter->first][i]);
                    }
                }
            }

            boost::mt19937 rngP1(rand());
            boost::mt19937 rngP2(rand());
            boost::uniform_int<> distP1(0, p1.size());
           
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genPoint1(rngP1, distP1);

            int pos1 = genPoint1();

            boost::uniform_int<> distP2(pos1, p2.size());
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genPoint2(rngP1, distP2);
            
            int pos2 = genPoint2();

            for(uint k = 0; k < pos1; ++k)
                c.push_back(p1[k]);

            for(uint k = pos1; k < pos2; ++k)
                c.push_back(p2[k]);

            for(uint k = pos2; k < p1.size(); ++k)
                c.push_back(p1[k]);

            int currentChildPosition = 0;
            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetworkWeights;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights;
                    for(uint i = 0; i < iter->second.size(); i++){
                        weights.push_back(c[currentChildPosition]);
                        currentChildPosition++;
                    }
                    currentNetworkWeights[iter->first] = weights;
                }
                childWeights.push_back(currentNetworkWeights);
            }
            Chromosome* child = parents[0]->clone();
            child->setWeights(childWeights);
            offspring.push_back(child);
        }
    }

    return offspring;
}