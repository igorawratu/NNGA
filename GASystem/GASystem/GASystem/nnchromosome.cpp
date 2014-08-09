#include "nnchromosome.h"

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

void NNChromosome::reInitialize(){
    cout << "reinitialise not implemented for NNChromosome yet" << endl;
}


NNChromosome& NNChromosome::operator = (const NNChromosome& other){
    mNets = other.mNets;

    return *this;
}

NNChromosome::NNChromosome(vector<NeuralNetwork> _nets){
    mNets = _nets;
}

bool NNChromosome::addDelta(vector<map<uint, vector<double>>> _weights){
    if(_weights.size() != mNets.size())
        return false;
    
    for(uint k = 0; k < _weights.size(); ++k){
        map<uint, vector<double>> nnWeights = mNets[k].getWeights();

        for(map<uint, vector<double>>::iterator iter = _weights[k].begin(); iter != _weights[k].end(); ++iter){
            for(uint i = 0; i < iter->second.size(); ++i){
                nnWeights[iter->first][i] += iter->second[i];
            }
        }

        mNets[k].setWeights(nnWeights);
    }

    return true;
}

void NNChromosome::mutate(string _mutationType, map<string, double>& _parameters){
    Mutation* mutationAlgorithm = MutationFactory::instance().create(_mutationType);

    for(uint k = 0; k < mNets.size(); k++){
        map<uint, vector<double>> currNetWeights = mNets[k].getWeights();
        for(map<uint, vector<double>>::iterator iter = currNetWeights.begin(); iter != currNetWeights.end(); iter++)
            mutationAlgorithm->execute(iter->second, _parameters);

        mNets[k].setWeights(currNetWeights);
    }

    delete mutationAlgorithm;
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