#ifndef NEURON_H
#define NEURON_H

#include <map>
#include <vector>
#include <math.h>
#include <iostream>
#include <set>

#include "common.h"

using namespace std;

const double e = 2.71828182845904523536;

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
    Neuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction){
        mNeuronCache = _neuronCache;
        mWeights = _weights;
        mCurrentCounter = -1;
        mLastOutput = 0;
        mActivationFunction = _activationFunction;
    }

    Neuron(const Neuron& _other){}
    virtual ~Neuron(){
        mNeuronCache = 0;
    }

    //gets the output of the neuron
    virtual double evaluate(long _counter)=0;

    //sets the inputs allowed for the neuron
    virtual bool setInput(set<uint> _inputs, bool _checkForLoops)=0;
    virtual bool setInput(double _inputs)=0;

    virtual bool checkLoop(Neuron* _loopNeuron)=0;

    vector<double> getWeights(){
        return mWeights;
    }

    void setWeights(vector<double> _weights){
        mWeights = _weights;
    }

    void setNeuronCache(map<uint, Neuron*> *_neuronCache){
        mNeuronCache = _neuronCache;
    }

    virtual Neuron* clone()=0;

    ActivationFunction getActivationFunction(){return mActivationFunction;}

    virtual NeuronType getNeuronType()=0;
    virtual set<uint> getPredecessors()=0;

protected:
    double calculateActivationEnergy(double _netSignal){
        double output;
        
        switch(mActivationFunction){
            case SIGMOID:
                output = 1/(1 + pow(e, -_netSignal));
                break;
            default:
                cout << "Error: unable to determine the activation function type" << endl;
                break;
        }

        return output;
    }
    Neuron(){}

protected:
    map<uint, Neuron*> *mNeuronCache;
    ActivationFunction mActivationFunction;
    double mLastOutput;
    long mCurrentCounter;
    vector<double> mWeights;
};

#endif