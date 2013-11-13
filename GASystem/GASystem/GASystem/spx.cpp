#include "spx.h"

SPX::SPX(){}
SPX::~SPX(){}

vector<Chromosome*> SPX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
    assert(_population.size() >= 4);
    
    vector<Chromosome*> offspring;

    while(offspring.size() < numOffspring){
        vector<Chromosome*> parents = _selectionAlgorithm->execute(_population, 3, vector<Chromosome*>());
        quicksort(parents, 0, parents.size() - 1);

        vector<map<uint, vector<double>>> centerOfMass, pFirstWeights, pLastWeights, childWeights;
        pFirstWeights = parents[0]->getWeightData(); pLastWeights = parents[parents.size() - 1]->getWeightData();

        //calc com here
        for(uint k = 0; k < pFirstWeights.size(); k++){
            map<uint, vector<double>> currentNetworkWeights;
            for(map<uint, vector<double>>::iterator iter = pFirstWeights[k].begin(); iter != pFirstWeights[k].end(); iter++){
                vector<double> weights;

                for(uint i = 0; i < iter->second.size(); i++){
                    double currCom = 0;
                    for(uint l = 0; l < parents.size() - 1; l++)
                        currCom += parents[l]->getWeightData()[k][iter->first][i];

                    weights.push_back(currCom / (parents.size() - 1));
                }
                currentNetworkWeights[iter->first] = weights;
            }
            centerOfMass.push_back(currentNetworkWeights);
        }

        for(uint k = 0; k < centerOfMass.size(); k++){
            map<uint, vector<double>> currentNetworkWeights;
            for(map<uint, vector<double>>::iterator iter = centerOfMass[k].begin(); iter != centerOfMass[k].end(); iter++){
                vector<double> weights;

                for(uint i = 0; i < iter->second.size(); i++)
                    weights.push_back(iter->second[i] + pFirstWeights[k][iter->first][i] - pLastWeights[k][iter->first][i]);

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

void SPX::quicksort(vector<Chromosome*>& elements, int left, int right)
{
    int i = left;
    int j = right;

    Chromosome* pivot = elements[(left+ right) / 2];
    do{
	    while (elements[i]->fitness() < pivot->fitness())
		    i++;
	    while (elements[j]->fitness() > pivot->fitness())
		    j--;

	    if (i <= j){
		    Chromosome* temp = elements[i]; elements[i] = elements[j]; elements[j] = temp;
		    i++; j--;
	    }
    }while (i <= j);

    if(left < j)
	    quicksort(elements, left, j);
    if(i < right)
	    quicksort(elements, i, right);
}