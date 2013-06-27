#ifndef LEAFNEURON_H
#define LEAFNEURON_H

#include <set>
#include <map>
#include <vector>

#include "neuron.h"

using namespace std;

class LeafNeuron : public Neuron
{
public:
    LeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction);
    LeafNeuron(const LeafNeuron& _other);
    LeafNeuron& operator = (const LeafNeuron& _other);
    ~LeafNeuron();
    
    virtual double evaluate(long _counter, vector<double> *_inputs);
    virtual void setInputs(set<uint> _inputs);

private:
    set<uint> mInputMask;

private:
    LeafNeuron(){}
};

#endif