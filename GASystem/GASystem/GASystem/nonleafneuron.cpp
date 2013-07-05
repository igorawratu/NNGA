#include "nonleafneuron.h"

NonLeafNeuron::NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction) : Neuron(_neuronCache, _weights, _activationFunction){
}

NonLeafNeuron::NonLeafNeuron(const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
}

NonLeafNeuron& NonLeafNeuron::operator = (const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;

    return *this;
}

NonLeafNeuron::~NonLeafNeuron(){
    mNeuronCache = 0;
    mPredecessors.clear();
}

double NonLeafNeuron::evaluate(long _counter){
    if(_counter == mCurrentCounter)
        return mLastOutput;
    else{
        mCurrentCounter = _counter;

        double netInputSignal = 0;

        uint k;

        for(set<uint>::iterator iter = mPredecessors.begin(); iter != mPredecessors.end(); iter++)
            netInputSignal += (*mNeuronCache)[*iter]->evaluate(_counter) * mWeights[k];

        netInputSignal += -1 * mWeights[k];

        return calculateActivationEnergy(netInputSignal);    
    }
}

bool NonLeafNeuron::checkLoop(Neuron* _loopNeuron){
    for(set<uint>::iterator iter = mPredecessors.begin(); iter != mPredecessors.end(); iter++)
        if((*mNeuronCache)[*iter] == _loopNeuron)
            return true;

    for(set<uint>::iterator iter = mPredecessors.begin(); iter != mPredecessors.end(); iter++)
        if((*mNeuronCache)[*iter]->checkLoop(_loopNeuron))
            return true;

    return false;
}

void NonLeafNeuron::setInput(set<uint> _inputs, bool _checkLoops){
    assert(_inputs.size() + 1 == mWeights.size());
    for(set<uint>::iterator iter = _inputs.begin(); iter != _inputs.end(); iter++){
        map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache->find(*iter);

        if(neuronIter == mNeuronCache->end())
        {
            mPredecessors.clear();
            cerr << "Error: Cannot find the neuron ID " << *iter << " to link to non-leaf neuron" << endl;
            return;
        }

        if(_checkLoops && (*mNeuronCache)[*iter]->checkLoop(this))
        {
            mPredecessors.clear();
            cerr << "Error: a loop was found when attempting to link the neuron with ID " << *iter <<  " to a non-leaf neuron" << endl;
            return;
        }
    }

    mPredecessors = _inputs;
}

void NonLeafNeuron::setInput(double _input){
    cerr << "Error: cannot set input value for non-leaf node" << endl;
}