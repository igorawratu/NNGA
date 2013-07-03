#ifndef LEAFNEURON_H
#define LEAFNEURON_H

#include <set>
#include <map>
#include <vector>
#include <assert.h>

#include "neuron.h"

using namespace std;

class LeafNeuron : public Neuron
{
public:
    LeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights);
    LeafNeuron(const LeafNeuron& _other);
    LeafNeuron& operator = (const LeafNeuron& _other);
    ~LeafNeuron();
    
    virtual double evaluate(long _counter);
    virtual void setInput(set<uint> _inputs, bool _checkForLoops);
    virtual void setInput(double _inputs);
    virtual bool checkLoop(Neuron* _loopNeuron);

private:
    double mInputValue;

private:
    LeafNeuron(){}
};

#endif