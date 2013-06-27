#include "leafneuron.h"

LeafNeuron::LeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weightse) : Neuron(_neuronCache, _weights, SIGMOID){
    mInputValue = 0;
}

LeafNeuron::LeafNeuron(const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mInputValue = _other.mInputValue;
}

LeafNeuron::LeafNeuron& operator = (const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mCurrentCounter = -1;
    mLastOutput = 0;
    mActivationFunction = _other.mActivationFunction;
    mInputValue = _other.mInputValue;

    return *this;
}
    
LeafNeuron::~LeafNeuron(){
    mNeuronCache = 0;
}
    
double LeafNeuron::evaluate(long _counter){
    return mInputValue;
}

void LeafNeuron::setInput(set<uint> _inputs, bool _checkForLoops){
    cerr << "Error: Cannot set a predecessor nodes for an input node" << endl;
}

void LeafNeuron::setInput(double _input){
    mInputValue = _input;
}

bool checkLoop(Neuron* _loopNeuron){
    return false;
}