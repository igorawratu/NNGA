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
    NeuralNetwork(char* _fileName);
    NeuralNetwork(const NeuralNetwork& _other);
    NeuralNetwork& operator = (const NeuralNetwork& _other);
    ~NeuralNetwork();

    vector<float> evaluate(map<uint, double> _inputs);
    void printStructure(char* _fileName);

private:
    void constructNNStructure(char* _fileName);

private:
    vector<Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;
    long mCounter;

private
    NeuralNetwork(){}
};

#endif