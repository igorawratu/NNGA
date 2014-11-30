#include "nonleafneuron.h"

NonLeafNeuron::NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction, uint _teamID) : Neuron(_neuronCache, _weights, _activationFunction, _teamID){
}

NonLeafNeuron::NonLeafNeuron(const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
	mTeamID = _other.mTeamID;
    mCurrentCounter = -1;
    mLastOutput = 0;
}

NonLeafNeuron& NonLeafNeuron::operator = (const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
    mLastOutput = 0;
    mCurrentCounter = -1;
	mTeamID = _other.mTeamID;

    return *this;
}

NonLeafNeuron::~NonLeafNeuron(){
    mNeuronCache = 0;
    mPredecessors.clear();
}

Neuron* NonLeafNeuron::clone(){
    return new NonLeafNeuron(*this);
}

double NonLeafNeuron::evaluate(long _counter){
    if(!mCurrentCounter != _counter){
        double netInputSignal = 0;

        uint k = 0;

        for(k = 0; k < mPredecessors.size(); ++k){
            netInputSignal += (*mNeuronCache)[mPredecessors[k]]->evaluate(_counter) * mWeights[k];
        }

        netInputSignal += -1 * mWeights[k];

        mLastOutput = calculateActivationEnergy(netInputSignal);
        mCurrentCounter = _counter;
    }


    return mLastOutput;
}

bool NonLeafNeuron::checkLoop(Neuron* _loopNeuron){
    for(uint k = 0; k < mPredecessors.size(); ++k)
        if((*mNeuronCache)[mPredecessors[k]] == _loopNeuron)
            return true;

    for(uint k = 0; k < mPredecessors.size(); ++k)
        if((*mNeuronCache)[mPredecessors[k]]->checkLoop(_loopNeuron))
            return true;

    return false;
}

bool NonLeafNeuron::setInput(vector<uint> _inputs, bool _checkLoops){
    assert(_inputs.size() + 1 == mWeights.size());
    if(mNeuronCache != NULL){
        for(uint k = 0; k < _inputs.size(); ++k){
            map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache->find(_inputs[k]);

            if(neuronIter == mNeuronCache->end())
            {
                mPredecessors.clear();
                cerr << "Error: Cannot find the neuron ID " << _inputs[k] << " to link to non-leaf neuron" << endl;
                return false;
            }

            if(_checkLoops && (*mNeuronCache)[_inputs[k]]->checkLoop(this))
            {
                mPredecessors.clear();
                cerr << "Error: a loop was found when attempting to link the neuron with ID " << _inputs[k] <<  " to a non-leaf neuron" << endl;
                return false;
            }
        }
    }

    mPredecessors = _inputs;

    return true;
}

bool NonLeafNeuron::setInput(double _input){
    cerr << "Error: cannot set input value for non-leaf node" << endl;
    return true;
}