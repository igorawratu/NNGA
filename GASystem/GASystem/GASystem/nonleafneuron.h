#ifndef NONLEAFNEURON_H
#define NONLEAFNEURON_H

#include <vector>
#include <map>
#include <assert.h>
#include <set>

#include "neuron.h"

using namespace std;

class NonLeafNeuron : public Neuron
{
public:
    NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction);
    NonLeafNeuron(const NonLeafNeuron& _other);
    NonLeafNeuron& operator = (const NonLeafNeuron& _other);
    ~NonLeafNeuron();
    
    virtual double evaluate(long _counter);
    virtual void setInput(set<uint> _inputs, bool _checkForLoops);
    virtual void setInput(double _inputs);
    virtual bool checkLoop(Neuron* _loopNeuron);

private:
    vector<Neuron*> mPredecessors;

private:
    NonLeafNeuron(){}
};

#endif