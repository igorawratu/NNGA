#include "neuron.h"

Neuron::Neuron(map<uint, Neuron*> *_neuronCache, vector<double> _weights, ActivationFunction _activationFunction, uint _teamID){
    mNeuronCache = _neuronCache;
    mWeights = _weights;
    mActivationFunction = _activationFunction;
    mLastOutput = 0;
    mCurrentCounter = -1;
	mTeamID = _teamID;
}

Neuron::Neuron(const Neuron& _other){}

Neuron::~Neuron(){
    mNeuronCache = 0;
}

vector<double> Neuron::getWeights(){
    return mWeights;
}

void Neuron::setWeights(vector<double> _weights){
    mWeights = _weights;
}

void Neuron::setNeuronCache(map<uint, Neuron*> *_neuronCache){
    mNeuronCache = _neuronCache;
}

ActivationFunction Neuron::getActivationFunction(){
    return mActivationFunction;
}

double Neuron::calculateActivationEnergy(double _netSignal){
    double output;
    
    switch(mActivationFunction){
        case SIGMOID:
            output = 1/(1 + pow(e, -_netSignal));
            break;
        default:
            cout << "Error: unable to determine the activation function type" << endl;
            break;
    }

    return output;
}

Neuron::Neuron(){}

uint Neuron::getTeamID(){
	return mTeamID;
}	

void Neuron::setTeamID(uint _teamID){
	mTeamID = _teamID;
}
