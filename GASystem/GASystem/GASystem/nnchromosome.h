#ifndef NNCHROMOSOME_H
#define NNCHROMOSOME_H

#include "chromosome.h"
#include "neuralnetwork.h"
#include "mutation.h"
#include "mutationfactory.h"
#include "common.h"

#include <map>
#include <vector>

using namespace std;

class NNChromosome : public Chromosome
{
public:
    NNChromosome();
    virtual bool initialize(pugi::xml_node* _root);
    NNChromosome(const NNChromosome& other);
    virtual ~NNChromosome();
    NNChromosome& operator = (const NNChromosome& other);

    virtual void mutate(string _mutationType, map<string, double>& _parameters);
    virtual vector<map<uint, vector<double>>> getWeightData();
    virtual vector<map<uint, NeuronInfo>> getFullStructureData();
    virtual void setWeights(vector<map<uint, vector<double>>>& _weights);
    virtual void setStructure(vector<map<uint, NeuronInfo>>& structure);
    virtual Chromosome* clone();
    vector<NeuralNetwork>& getNeuralNets(){return mNets;}

private:
    vector<NeuralNetwork> mNets;

};

#endif