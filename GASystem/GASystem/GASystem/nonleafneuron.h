#ifndef NONLEAFNEURON_H
#define NONLEAFNEURON_H

#include <vector>
#include <map>

#include "neuron.h"

using namespace std;

class NonLeafNeuron : public Neuron
{
public:
    NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction);
    NonLeafNeuron(const NonLeafNeuron& _other);
    NonLeafNeuron& operator = (const NonLeafNeuron& _other);
    ~NonLeafNeuron();
    
    virtual double evaluate(long _counter, vector<double> *_inputs);
    virtual void setInputs(set<uint> _inputs);

private:
    vector<Neuron*> mPredecessors;

private:
    NonLeafNeuron(){}
};

#endif