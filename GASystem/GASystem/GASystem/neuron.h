#ifndef NEURON_H
#define NEURON_H

#include <map>
#include <vector>
#include <math.h>
#include <iostream>
#include <set>
#include <omp.h>

#include "common.h"

using namespace std;

enum ActivationFunction{SIGMOID};
enum NeuronType{LEAF, NONLEAF, OUTPUT};

struct NeuronInfo
{
    NeuronType neuronType;
    ActivationFunction activationFunction;
    set<uint> predecessors;
    vector<double> weights;
};

class Neuron
{
public:
    Neuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction);
    Neuron(const Neuron& _other);
    virtual ~Neuron();

    //gets the output of the neuron
    virtual double evaluate(long _counter)=0;

    //sets the inputs allowed for the neuron
    virtual bool setInput(set<uint> _inputs, bool _checkForLoops)=0;
    virtual bool setInput(double _inputs)=0;

    virtual bool checkLoop(Neuron* _loopNeuron)=0;

    vector<double> getWeights();

    void setWeights(vector<double> _weights);

    void setNeuronCache(map<uint, Neuron*> *_neuronCache);

    virtual Neuron* clone()=0;

    ActivationFunction getActivationFunction();

    virtual NeuronType getNeuronType()=0;
    virtual set<uint> getPredecessors()=0;

protected:
    double calculateActivationEnergy(double _netSignal);
    Neuron();

protected:
    map<uint, Neuron*> *mNeuronCache;
    ActivationFunction mActivationFunction;
    double mLastOutput;
    long mCurrentCounter;
    vector<double> mWeights;
};

#endif