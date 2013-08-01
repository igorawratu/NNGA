#include "nnchromosome.h"

vector<NeuralNetwork> mNets;

NNChromosome::NNChromosome(){

}

NNChromosome::NNChromosome(const NNChromosome& other){
    mNets = other.mNets;
}

bool NNChromosome::initialize(pugi::xml_node* _root){
    for(pugi::xml_node currNetwork = _root->first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
        NeuralNetwork currNN;
        if(!currNN.initialize(&currNetwork, true))
            return false;
        mNets.push_back(currNN);
    }
    return true;
}


NNChromosome::~NNChromosome(){

}

NNChromosome& NNChromosome::operator = (const NNChromosome& other){
    mNets = other.mNets;

    return *this;
}

void NNChromosome::mutate(string _mutationType, map<string, double>& _parameters){
    Mutation* mutationAlgorithm = AlgorithmCreator::instance().createMutationAlgorithm(_mutationType);

    for(uint k = 0; k < mNets.size(); k++){
        map<uint, vector<double>> currNetWeights = mNets[k].getWeights();
        for(map<uint, vector<double>>::iterator iter = currNetWeights.begin(); iter != currNetWeights.end(); iter++)
            mutationAlgorithm->execute(iter->second, _parameters);

        mNets[k].setWeights(currNetWeights);
    }
}

vector<map<uint, vector<double>>> NNChromosome::getWeightData(){
    vector<map<uint, vector<double>>> output;

    for(uint k = 0; k < mNets.size(); k++)
        output.push_back(mNets[k].getWeights());

    return output;
}

vector<map<uint, NeuronInfo>> NNChromosome::getFullStructureData(){
    vector<map<uint, NeuronInfo>> output;

    for(uint k = 0; k < mNets.size(); k++)
        output.push_back(mNets[k].getMapStructure());

    return output;
}

void NNChromosome::setWeights(vector<map<uint, vector<double>>>& _weights){
    for(uint k = 0; k < _weights.size(); k++){
        mNets[k].setWeights(_weights[k]);
    }
}   

void NNChromosome::setStructure(vector<map<uint, NeuronInfo>>& _structure){
    mNets.clear();
    for(uint k = 0; k < _structure.size(); k++)
        mNets.push_back(NeuralNetwork(_structure[k]));
}

Chromosome* NNChromosome::clone(){
    return new NNChromosome(*this);
}