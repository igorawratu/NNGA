#include "nonleafneuron.h"

NonLeafNeuron::NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction) : Neuron(_neuronCache, _weights, _activationFunction){
    mNeuronType = NONLEAF;
}

NonLeafNeuron::NonLeafNeuron(const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
    mNeuronType = NONLEAF;
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

        for(k = 0; k < mPredecessors.size(); k++)
            netInputSignal += mPredecessors[k]->evaluate(_counter) * mWeights[k];

        netInputSignal += -1 * mWeights[k];

        return calculateActivationEnergy(netInputSignal);    
    }
}

bool NonLeafNeuron::checkLoop(Neuron* _loopNeuron){
    for(int k = 0; k < mPredecessors.size(); k++)
        if(mPredecessors[k] == _loopNeuron)
            return true;

    for(int k = 0; k < mPredecessors.size(); k++)
        if(mPredecessors[k]->checkLoop(_loopNeuron))
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

        if(_checkLoops && mNeuronCache[*iter]->checkLoop(this))
        {
            mPredecessors.clear();
            cerr << "Error: a loop was found when attempting to link the neuron with ID " << *iter <<  " to a non-leaf neuron" << endl;
            return;
        }
        
        mPredecessors.push_back(mNeuronCache[*iter]);
    }
}

void NonLeafNeuron::setInput(double _input){
    cerr << "Error: cannot set input value for non-leaf node" << endl;
}