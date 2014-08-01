#include "nonleafneuron.h"

NonLeafNeuron::NonLeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction, uint _teamID) : Neuron(_neuronCache, _weights, _activationFunction, _teamID){
}

NonLeafNeuron::NonLeafNeuron(const NonLeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mActivationFunction = _other.mActivationFunction;
    mPredecessors = _other.mPredecessors;
	mTeamID = _other.mTeamID;
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
    //if(!mCurrentCounter != _counter){
        mCurrentCounter = _counter;

        double netInputSignal = 0;

        uint k = 0;

        for(set<uint>::iterator iter = mPredecessors.begin(); iter != mPredecessors.end(); iter++){
            netInputSignal += (*mNeuronCache)[*iter]->evaluate(_counter) * mWeights[k];
            k++;
        }

        netInputSignal += -1 * mWeights[k];

        mLastOutput = calculateActivationEnergy(netInputSignal);
    //}
    return mLastOutput;
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

bool NonLeafNeuron::setInput(set<uint> _inputs, bool _checkLoops){
    assert(_inputs.size() + 1 == mWeights.size());
    if(mNeuronCache != NULL){
        for(set<uint>::iterator iter = _inputs.begin(); iter != _inputs.end(); iter++){
            map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache->find(*iter);

            if(neuronIter == mNeuronCache->end())
            {
                mPredecessors.clear();
                cerr << "Error: Cannot find the neuron ID " << *iter << " to link to non-leaf neuron" << endl;
                return false;
            }

            if(_checkLoops && (*mNeuronCache)[*iter]->checkLoop(this))
            {
                mPredecessors.clear();
                cerr << "Error: a loop was found when attempting to link the neuron with ID " << *iter <<  " to a non-leaf neuron" << endl;
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