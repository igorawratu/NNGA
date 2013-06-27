#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork(char* _fileName){
    mCounter = -1;

    
}

void NeuralNetwork::constructNNStructure(char* _fileName){

}

NeuralNetwork::NeuralNetwork(const NeuralNetwork& _other){
    
}

NeuralNetwork& NeuralNetwork::operator = (const NeuralNetwork& _other){

}
    
NeuralNetwork::~NeuralNetwork(){
    for(map<uint, double>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second)
        {
            delete iter->second;
            iter->second = 0;
        }
    }
    mNeuronCache.clear();

    mOutput.clear();
}

vector<double> NeuralNetwork::evaluate(map<uint, double> _inputs){
    vector<double> output;

    for(map<uint, double>::iterator iter = _inputs.begin(); iter != _inputs.end(); iter++){
        map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache->find(*iter);
        if(neuronIter == mNeuronCache.end())
        {
            cerr << "Error: unable to find input node " << *iter << ", neuron evaluation will now terminate" << endl;
            return output;
        }

        mNeuronCache[iter->first]->setInput(iter->second);
    }

    mCounter++;
    for(int k = 0; k < mOutput.size(); k++)
        output.push_back(mOutput[k]->evaluate(mCounter));

    return output;
}

void NeuralNetwork::printStructure(char* _fileName){

}