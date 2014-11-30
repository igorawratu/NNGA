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
    NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction, uint _teamID);
    NonLeafNeuron(const NonLeafNeuron& _other);
    NonLeafNeuron& operator = (const NonLeafNeuron& _other);
    ~NonLeafNeuron();
    
    virtual double evaluate(long _counter);
    virtual bool setInput(vector<uint> _inputs, bool _checkForLoops);
    virtual bool setInput(double _inputs);
    virtual bool checkLoop(Neuron* _loopNeuron);
    virtual NeuronType getNeuronType(){return NONLEAF;}
    virtual vector<uint> getPredecessors(){return mPredecessors;}
    virtual Neuron* clone();

private:
    vector<uint> mPredecessors;

private:
    NonLeafNeuron(){}
};

#endif