#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <vector>
#include <map>
#include <cstring>

#include "common.h"
#include "neuron.h"
#include "leafneuron.h"
#include "nonleafneuron.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

using namespace std;

struct NeuronInfo
{

};

class NeuralNetwork
{
public:
    //actually pass an xml file here
    NeuralNetwork(xmldoc* _file, bool _checkLoops);
    NeuralNetwork(map<uint, NeuronInfo> _neuronInfo);
    NeuralNetwork(const NeuralNetwork& _other);
    NeuralNetwork& operator = (const NeuralNetwork& _other);
    ~NeuralNetwork();

    vector<double> evaluate(map<uint, double> _inputs);
    void setWeights(map<uint, vector<double>> _weights);
    void getStructure(xmldoc& _doc);

private:
    void constructNNStructure(xmldoc* _file, bool _withWeights, bool _checkLoops);

private:
    map<uint, Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;
    long mCounter;

private:
    NeuralNetwork(){}
};

#endif