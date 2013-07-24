#include "nnchromosome.h"

NNChromosome::NNChromosome(){

}

NNChromosome::NNChromosome(const NNChromosome& other){

}

NNChromosome::~NNChromosome(){

}

NNChromosome& NNChromosome::operator = (const NNChromosome& other){

}

void NNChromosome::mutate(string _mutationType){

}

vector<map<uint, vector<double>>> NNChromosome::getWeightData(){

}

vector<map<uint, NeuronInfo>> NNChromosome::getFullStructureData(){

}

void NNChromosome::setWeights(vector<map<uint, vector<double>>>){

}

void NNChromosome::setStructure(vector<map<uint, NeuronInfo>>){

}

Chromosome* NNChromosome::clone(){

}