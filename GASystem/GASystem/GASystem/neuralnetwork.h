#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <vector>
#include <map>

#include "common.h"
#include "neuron.h"

using namespace std;

class NeuralNetwork
{
public:
    //actually pass an xml file here
    NeuralNetwork(NNParameters _params);
    NeuralNetwork(char* _structure);
    NeuralNetwork(const NeuralNetwork& _other);
    ~NeuralNetwork();

    vector<float> evaluate(vector<float> _inputs);
    char* getStructure();

private:
    vector<Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;
    long counter;

private
    NeuralNetwork(){}
};

#endif