#ifndef NEURON_H
#define NEURON_H

#include <map>
#include <vector>

using namespace std;

enum ActivationFunction{SIGMOID};

class Neuron
{
public:
    Neuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction, ActivationFunction _activationFunction) {
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
    virtual double evaluate(long _counter, vector<double> *_inputs)=0;

    //sets the inputs allowed for the neuron
    virtual void setInputs(set<uint> _inputs)=0;

    //returns a reference to the weights of the neuron
    vector<double>& getWeights() {
        return mWeights;
    }

protected:
    double calculateActivationEnergy(double _netSignal)
    {
        double output;
        


        return output;
    }

protected:
    map<uint, Neuron*> *mNeuronCache;
    ActivationFunction mActivationFunction;
    double mLastOutput;
    long mCurrentCounter;
    vector<double> mWeights;
};

#endif