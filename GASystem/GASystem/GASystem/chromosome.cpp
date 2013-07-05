#include "chromosome.h"

Chromosome::Chromosome(char* _file){

}

Chromosome(char* _file, vector<map<uint, vector<double>>>){

}

Chromosome::Chromosome(vector<xmldoc>& format){
    for(int k = 0; k < format.size(); k++)
        mNeuralNetworks.push_back(NeuralNetwork(&format[k], false));
}

Chromosome::Chromosome(const Chromosome& other){
    mNeuralNetworks = other.mNeuralNetworks;
}

Chromosome& Chromosome::operator = (const Chromosome& other){

}

Chromosome::~Chromosome(){

}

vector<double> Chromosome::evaluateNN(uint _neuralNetID){

}

void Chromosome::writeToFiles(char* _file){

}