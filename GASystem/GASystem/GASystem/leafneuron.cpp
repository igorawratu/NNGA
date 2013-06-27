#include "leafneuron.h"

LeafNeuron::LeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction) : Neuron(_neuronCache, _weights, _activationFunction){
    
}

LeafNeuron::LeafNeuron(const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mInputMask = _other.mInputMask;
}

LeafNeuron::LeafNeuron& operator = (const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;

    return *this;
}
    
LeafNeuron::~LeafNeuron(){
    mNeuronCache = 0;
}
    
double LeafNeuron::evaluate(long _counter, vector<double> *_inputs){
    if(_counter == mCurrentCounter)
        return mLastOutput;
    else
    {
        assert(mInputMask.size() > 0);
        assert(mInputMask.size() < _inputs->size());
        assert(mWeights.size() == _inputs->size() + 1);
        
        mCurrentCounter = _counter;

        double netInputSignal = 0;

        uint k;

        for(set<uint>::iterator iter = mInputMask.begin(), k = 0; iter != mInputMask.end(); ++iter, k++)
            netInputSignal += *_inputs[*iter] * mWeights[k];

        netInputSignal += -1 * mWeights[k];

        return calculateActivationEnergy(netInputSignal);    
    }
}

void LeafNeuron::setInputs(set<uint> _inputs){
    mInputMask = _inputs;
}