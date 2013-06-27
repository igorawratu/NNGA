#include "nonleafneuron.h"

NonLeafNeuron::NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction) : Neuron(_neuronCache, _weights, _activationFunction)
{
}

NonLeafNeuron::NonLeafNeuron(const NonLeafNeuron& _other)
{
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
}

NonLeafNeuron::NonLeafNeuron& operator = (const NonLeafNeuron& _other)
{
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;

    return *this;
}

NonLeafNeuron::~NonLeafNeuron()
{
    mNeuronCache = 0;
    mPredecessors.clear();
}

double NonLeafNeuron::evaluate(long _counter, vector<double> *_inputs)
{
    if(_counter == mCurrentCounter)
        return mLastOutput;
    else
    {
        mCurrentCounter = _counter;

        double netInputSignal = 0;

        uint k;

        for(k = 0; k < mPredecessors.size(); k++)
            netInputSignal += mPredecessors[k]->evaluate(_inputs) * mWeights[k];

        netInputSignal += -1 * mWeights[k];

        return calculateActivationEnergy(netInputSignal);    
    }
}

void NonLeafNeuron::setInputs(set<uint> _inputs)
{
    assert(_inputs.size() + 1 == mWeights.size());
    for(set<uint>::iterator iter = _inputs.begin(); iter != _inputs.end(); iter++)
    {
        map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache->find(name);
        if(neuronIter == mNeuronCache->end())
        {
            mPredecessors.clear();
            cerr << "Error: Cannot find the neuron ID " << *iter << " to link to non-leaf neuron" << endl;
            return;
        }
        
        mPredecessors.push_back(mNeuronCache[*iter]);
    }
}