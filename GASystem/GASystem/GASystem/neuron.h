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
enum NeuronType{LEAF, NONLEAF};

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
    virtual void setInput(set<uint> _inputs, bool _checkForLoops)=0;
    virtual void setInput(double _inputs)=0;

    virtual bool checkLoop(Neuron* _loopNeuron)=0;

    //returns a reference to the weights of the neuron
    vector<double>& weights(){
        return mWeights;
    }

    NeuronType getNeuronType(){return mNeuronType;}

protected:
    double calculateActivationEnergy(double _netSignal){
        double output;
        
        switch(mActivationFunction){
            case SIGMOID:
                output = 1/(1 + pow(e, _netSignal));
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
    NeuronType mNeuronType;
};

#endif