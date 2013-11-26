#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <vector>
#include <map>
#include <cstring>

#include "common.h"
#include "neuron.h"
#include "leafneuron.h"
#include "nonleafneuron.h"

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

class NeuralNetwork
{
public:
    //creates an empty shell, need to call initializeNN for the actual neural network to be constructed, otherwise call another constructor/copy constructor
    NeuralNetwork();
    NeuralNetwork(map<uint, NeuronInfo> _neuronInfo);
    NeuralNetwork(const NeuralNetwork& _other);
    NeuralNetwork& operator = (const NeuralNetwork& _other);
    ~NeuralNetwork();

    //creates the neural network after calling the default constructor, calling any other constructor/copy constructor will fully create the neural network, the reason this function is used is 
    //to allow one to check if the actual construction of the neural net has been successful
    bool initialize(pugi::xml_node* _nnRoot, bool _checkLoops);

    vector<double> evaluate(map<uint, double> _inputs);
    void setWeights(map<uint, vector<double>> _weights);
    void setStructure(map<uint, NeuronInfo> _neuronInfo);
    void setStructure(map<uint, Neuron*> _neuronCache, map<uint, Neuron*> _output);
    void getXMLStructure(pugi::xml_node& _root);
    map<uint, NeuronInfo> getMapStructure();
    map<uint, vector<double>> getWeights();

private:
    bool constructNNStructure(pugi::xml_node* _file, bool _checkLoops);

private:
    map<uint, Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;
    long mCounter;
};

#endif