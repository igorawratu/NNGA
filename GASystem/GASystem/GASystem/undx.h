#ifndef UNDX_H
#define UNDX_H

#include "crossover.h"
#include "selectionfactory.h"
#include "chromosome.h"
#include "common.h"

#include <vector>
#include <map>
#include <cmath>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

class UNDX : public Crossover
{
public:
    UNDX(){}
    virtual ~UNDX(){}

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters){
        Selection* selectionAlgorithm = SelectionFactory::instance().create("RankSelection");
        if(!selectionAlgorithm)
            return _population;
        
        vector<Chromosome*> offspring;

        boost::mt19937 rng(rand());
        boost::uniform_real<double> dist(0, 1);
        boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

        uint dimensions = 0;
        
        vector<map<uint, vector<double>>> weights = _population[0]->getWeightData();
        for(uint k = 0; k < weights.size(); k++){
            for(map<uint, vector<double>>::iterator iter = weights[k].begin(); iter != weights[k].end(); iter++)
                for(uint i = 0; i < iter->second.size(); i++)
                    dimensions++;


        while(offspring.size() < numOffspring){
            vector<Chromosome*> parents = selectionAlgorithm->execute(_population, 3, vector<Chromosome*>());

            vector<map<uint, vector<double>>> p1Weights, p2Weights, p3Weights, c1Weights, c2Weights;
            vector<map<uint, vector<double>>> d, m;

            p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData(); p3Weights = parents[2]->getWeightData();
            
            for(uint k = 0; k < p1Weights.size(); k++){
                d.push_back(map<uint, vector<double>>());
                m.push_back(map<uint, vector<double>>());
                
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    d[k][iter->first] = vector<double>();
                    m[k][iter->first] = vector<double>();

                    for(uint i = 0; i < iter->second.size(); i++){
                        d[k][iter->first].push_back(fabs(p1Weights[k][iter->first][i] - p2Weights[k][iter->first][i]));
                        m[k][iter->first].push_back((p1Weights[k][iter->first][i] + p2Weights[k][iter->first][i]) / 2);
                    }
                }
            }

            vector<vector<map<uint, vector<double>>>> obvecs;
            //calculate orthonormal basis vectors of d here

            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> c1CurrentNetworkWeights, c2CurrentNetworkWeights;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> c1weights, c2Weights;

                    for(uint i = 0; i < iter->second.size(); i++){
                        
                    }

                    c1CurrentNetworkWeights[iter->first] = c1weights;
                    c2CurrentNetworkWeights[iter->first] = c2weights;
                }
                c1Weights.push_back(c1CurrentNetworkWeights);
                c2Weights.push_back(c2CurrentNetworkWeights);
            }
            Chromosome* child1 = parents[0]->clone();
            Chromosome* child2 = parents[0]->clone();
            
            child1->setWeights(c1Weights);
            child2->setWeights(c2Weights);

            offspring.push_back(child1);
            offspring.push_back(child2);
        }


        delete selectionAlgorithm;

        return offspring;
    }

    static Crossover* createUNDX(){
        return new UNDX();
    }

};

#endif