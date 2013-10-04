#include "leafneuron.h"

LeafNeuron::LeafNeuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights) : Neuron(_neuronCache, _weights, SIGMOID){
}

LeafNeuron::LeafNeuron(const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mActivationFunction = _other.mActivationFunction;
}

LeafNeuron& LeafNeuron::operator = (const LeafNeuron& _other){
    mNeuronCache = _other.mNeuronCache;
    mWeights = _other.mWeights;
    mActivationFunction = _other.mActivationFunction;

    return *this;
}
    
LeafNeuron::~LeafNeuron(){
    mNeuronCache = 0;
}

Neuron* LeafNeuron::clone(){
    return new LeafNeuron(*this);
}
    
double LeafNeuron::evaluate(long _counter){
    return mLastOutput;
}

bool LeafNeuron::setInput(set<uint> _inputs, bool _checkForLoops){
    cerr << "Error: Cannot set a predecessor nodes for an input node" << endl;
    return true;
}

bool LeafNeuron::setInput(double _input){
    mLastOutput = _input;

    return true;
}

bool LeafNeuron::checkLoop(Neuron* _loopNeuron){
    return false;
}