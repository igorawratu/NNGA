#ifndef NNCHROMOSOME_H
#define NNCHROMOSOME_H

#include "chromosome.h"
#include "neuralnetwork.h"
#include <vector>

using namespace std;


class NNChromosome : public Chromosome
{
public:
    NNChromosome();
    NNChromosome(const NNChromosome& other);
    virtual ~NNChromosome();
    NNChromosome& operator = (const NNChromosome& other);

    virtual void mutate(string _mutationType);
    virtual vector<map<uint, vector<double>>> getWeightData();
    virtual vector<map<uint, NeuronInfo>> getFullStructureData();
    virtual void setWeights(vector<map<uint, vector<double>>>);
    virtual void setStructure(vector<map<uint, NeuronInfo>>);
    virtual Chromosome* clone();

private:
    vector<NeuralNetwork> nets;

};

#endif